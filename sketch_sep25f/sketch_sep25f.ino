#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// --- KONFIGURASI PIN UART ESP32-C3 (SESUAI JALUR PCB) ---
#define RX_PIN 7  // Terhubung ke Pin 20 (TX_TELEMETRY) di ESP32-S3 ECU[cite: 1, 2]
#define TX_PIN 6  // Terhubung ke Pin 19 (RX_TELEMETRY) di ESP32-S3 ECU[cite: 1, 2]

// --- MAC ADDRESS TRANSMITTER & DISPLAY ---
uint8_t transmitterMac[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
uint8_t displayMacAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#define NUM_RPM_POINTS 15
#define NUM_TPS_POINTS 5
Preferences preferences;
AsyncWebServer server(80);

// --- VARIABEL EFISIENSI & TIMEOUT WIFI (5 MENIT) ---
unsigned long lastActivityTime = 0;
const unsigned long WIFI_TIMEOUT_MS = 300000; 
bool isWifiActive = true;

// --- STATUS KONEKSI UART & SINKRONISASI ---
unsigned long lastUartRxTime = 0;
const unsigned long UART_TIMEOUT_MS = 2000;
bool isEcuConnected = false;
bool initialSyncDone = false;
unsigned long lastSyncRetryTime = 0;
uint8_t syncRetryCount = 0;

// --- DATA LOGGER CONFIGURATION (RAM RING BUFFER) ---
#define MAX_LOG_ENTRIES 800

struct LogEntry {
  uint32_t timestampMs;
  uint16_t rpm;
  uint8_t  tps;
  int16_t  degree10;
  int16_t  temp10;
  uint16_t battVolt10;
};

LogEntry logBuffer[MAX_LOG_ENTRIES];
uint16_t logHead = 0;
uint16_t logCount = 0;
bool bufferOverflowed = false; 
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

// --- DATA BAWAAN DEFAULT ---
const int16_t mapDaily3DBase[NUM_RPM_POINTS][NUM_TPS_POINTS] = {
  {100, 100, 100, 100, 100}, {100, 100, 100, 100, 100}, {140, 150, 160, 170, 180},
  {180, 200, 220, 230, 240}, {220, 240, 260, 270, 280}, {250, 270, 290, 300, 310},
  {280, 300, 320, 330, 330}, {300, 320, 340, 350, 350}, {310, 330, 350, 350, 350},
  {310, 330, 350, 350, 350}, {300, 320, 340, 340, 340}, {280, 300, 320, 320, 320},
  {260, 280, 300, 300, 300}, {250, 250, 280, 280, 280}, {250, 250, 250, 250, 250}
};

const uint16_t defaultDwellPerRPM[NUM_RPM_POINTS] = {
  3500, 3500, 3400, 3300, 3200, 3100, 3000, 2900, 2800, 2700, 2600, 2500, 2400, 2300, 2200
};

struct CustomTuning {
  int16_t mapData[NUM_RPM_POINTS][NUM_TPS_POINTS];
  uint16_t dwellData[NUM_RPM_POINTS]; 
  uint16_t rpmLimit;
};
CustomTuning customMapSlots[5];
uint8_t currentActiveSlot = 0; 

// --- STRUKTUR PAKET KOMUNIKASI ---
struct __attribute__((packed)) TelemetryData {
  uint16_t header;     // 0xAA55
  uint16_t rpm;
  uint8_t  tps;        
  int16_t  degree10;   
  int16_t  temp10;     
  uint16_t battVolt10; 
  uint8_t  mode;
  uint8_t  fuel;
  uint8_t  state;
  uint8_t  ackStatus;  
  uint16_t faultCode;  
  uint16_t crc16;
};

struct __attribute__((packed)) CommandData {
  uint16_t header;     // 0xCC55
  uint8_t requestedMode;
  uint16_t crc16;
};

struct __attribute__((packed)) CommandPacketToECU {
  uint16_t header;                                  // 0x55CC
  uint8_t  cmdType;                                 // 0x01: Mode, 0x02: RAM Map, 0x03: Save NVS
  uint8_t  slotOrMode;   
  int16_t  mapData[NUM_RPM_POINTS][NUM_TPS_POINTS]; 
  uint16_t dwellData[NUM_RPM_POINTS];               
  uint16_t rpmLimit;                                
  uint16_t crc16;                                   
};

portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
TelemetryData currentTelemetry = {0xAA55, 0, 0, 100, 300, 126, 0, 0, 0, 0, 0, 0};

uint16_t calculateCRC16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021; else crc <<= 1;
    }
  }
  return crc;
}

void pushToLog(const TelemetryData *pkt) {
  if (pkt->rpm < 400) return;
  portENTER_CRITICAL(&logMux);
  logBuffer[logHead] = (LogEntry){
    .timestampMs = millis(),
    .rpm = pkt->rpm,
    .tps = pkt->tps,
    .degree10 = pkt->degree10,
    .temp10 = pkt->temp10,
    .battVolt10 = pkt->battVolt10
  };
  logHead = (logHead + 1) % MAX_LOG_ENTRIES;
  if (logCount < MAX_LOG_ENTRIES) {
    logCount++;
  } else {
    bufferOverflowed = true;
  }
  portEXIT_CRITICAL(&logMux);
}

void sendCommandToECU(uint8_t cmdType, uint8_t slotOrMode) {
  CommandPacketToECU cmd;
  cmd.header = 0x55CC;
  cmd.cmdType = cmdType;
  cmd.slotOrMode = slotOrMode;
  
  if ((cmdType == 0x02 || cmdType == 0x03) && slotOrMode < 5) {
    memcpy(cmd.mapData, customMapSlots[slotOrMode].mapData, sizeof(cmd.mapData));
    memcpy(cmd.dwellData, customMapSlots[slotOrMode].dwellData, sizeof(cmd.dwellData));
    cmd.rpmLimit = customMapSlots[slotOrMode].rpmLimit;
  } else {
    memset(cmd.mapData, 0, sizeof(cmd.mapData)); 
    memcpy(cmd.dwellData, defaultDwellPerRPM, sizeof(defaultDwellPerRPM));
    cmd.rpmLimit = 12500;
  }
  
  cmd.crc16 = calculateCRC16((uint8_t*)&cmd, sizeof(CommandPacketToECU) - sizeof(uint16_t));
  Serial1.write((uint8_t*)&cmd, sizeof(CommandPacketToECU));
}

void parseUART() {
  static uint8_t rxBuffer[sizeof(TelemetryData)];
  static size_t rxIndex = 0;

  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    if (rxIndex == 0) { if (c == 0xAA) rxBuffer[rxIndex++] = c; continue; } 
    else if (rxIndex == 1) {
      if (c == 0x55) rxBuffer[rxIndex++] = c; else rxIndex = (c == 0xAA) ? 1 : 0; continue;
    }
    rxBuffer[rxIndex++] = c;

    if (rxIndex >= sizeof(TelemetryData)) {
      TelemetryData *pkt = (TelemetryData*)rxBuffer;
      if (pkt->crc16 == calculateCRC16(rxBuffer, sizeof(TelemetryData) - sizeof(uint16_t))) {
        portENTER_CRITICAL(&dataMux);
        memcpy(&currentTelemetry, pkt, sizeof(TelemetryData));
        portEXIT_CRITICAL(&dataMux);
        
        lastUartRxTime = millis();
        if (!isEcuConnected) {
          isEcuConnected = true;
          initialSyncDone = false;
          syncRetryCount = 0;
          lastSyncRetryTime = 0;
        }
        
        if (!initialSyncDone && syncRetryCount > 0 && pkt->ackStatus == 1) {
          initialSyncDone = true;
        }
        
        pushToLog(pkt); 
        esp_now_send(displayMacAddress, rxBuffer, sizeof(TelemetryData));
      }
      rxIndex = 0;
    }
  }
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnEspNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {}
void OnEspNowRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len) {
#else
void OnEspNowSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}
void OnEspNowRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif
  if (len == sizeof(CommandData)) {
    CommandData *cmd = (CommandData*)incomingData;
    uint16_t calcCrc = calculateCRC16(incomingData, sizeof(CommandData) - sizeof(uint16_t));
    if (cmd->header == 0xCC55 && cmd->crc16 == calcCrc) {
      sendCommandToECU(0x01, cmd->requestedMode); 
    }
  }
}

void loadMapFromNVS() {
  preferences.begin("ecu_tuning", false);
  currentActiveSlot = preferences.getUChar("activeSlot", 0);
  if (currentActiveSlot > 4) currentActiveSlot = 0;

  if (preferences.getBytesLength("customMaps") == sizeof(customMapSlots)) {
    preferences.getBytes("customMaps", &customMapSlots, sizeof(customMapSlots));
  } else {
    for (int s = 0; s < 5; s++) {
      customMapSlots[s].rpmLimit = 12500;
      memcpy(customMapSlots[s].dwellData, defaultDwellPerRPM, sizeof(defaultDwellPerRPM));
      memcpy(customMapSlots[s].mapData, mapDaily3DBase, sizeof(mapDaily3DBase));
    }
  }
  preferences.end();
}

void saveMapToNVS() {
  preferences.begin("ecu_tuning", false);
  preferences.putUChar("activeSlot", currentActiveSlot);
  preferences.putBytes("customMaps", &customMapSlots, sizeof(customMapSlots));
  preferences.end();
}

const char* htmlUI = R"rawliteral(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>PATAS KUDUS — S3 TCI TELEMETRY PRO</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Chakra+Petch:ital,wght@0,600;0,700;1,700&family=Orbitron:wght@600;800;900&display=swap');
  :root {
    --bg-dark: #06070a;
    --bg-panel: #0e1118;
    --bg-glass: rgba(18, 22, 32, 0.75);
    --border-subtle: rgba(255, 255, 255, 0.08);
    --accent-cyan: #00f0ff;
    --accent-green: #00ff88;
    --accent-amber: #ffb800;
    --accent-red: #ff1e56;
    --text-primary: #f4f6fb;
    --text-dim: #7b849b;
  }
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
  body {
    font-family: 'Chakra Petch', sans-serif;
    background-color: var(--bg-dark);
    background-image: 
      radial-gradient(circle at 50% 0%, rgba(0, 240, 255, 0.08) 0%, transparent 60%),
      linear-gradient(rgba(255,255,255,0.015) 1px, transparent 1px),
      linear-gradient(90deg, rgba(255,255,255,0.015) 1px, transparent 1px);
    background-size: 100% 100%, 24px 24px, 24px 24px;
    color: var(--text-primary);
    margin: 0; padding: 0; overflow-x: hidden;
  }

  /* Animated Ticker Marquee */
  .marquee-strip {
    background: linear-gradient(90deg, #ff1e56, #00f0ff, #00ff88, #ff1e56);
    background-size: 300% 300%;
    animation: gradientShift 8s ease infinite;
    padding: 2px 0;
  }
  @keyframes gradientShift { 0% { background-position: 0% 50%; } 50% { background-position: 100% 50%; } 100% { background-position: 0% 50%; } }
  .marquee-inner { background: #000; overflow: hidden; white-space: nowrap; padding: 4px 0; }
  .marquee-text {
    display: inline-block; font-family: 'Orbitron', sans-serif; font-size: 11px; font-weight: 800;
    color: var(--accent-green); text-transform: uppercase; animation: marqueeScroll 20s linear infinite;
    letter-spacing: 2px; text-shadow: 0 0 8px rgba(0,255,136,0.6);
  }
  @keyframes marqueeScroll { 0% { transform: translateX(100%); } 100% { transform: translateX(-100%); } }

  /* Top Racing Header */
  .top-bar {
    background: rgba(10, 12, 18, 0.95);
    backdrop-filter: blur(12px);
    border-bottom: 1px solid var(--border-subtle);
    padding: 10px 14px;
    display: flex; justify-content: space-between; align-items: center;
    position: sticky; top: 0; z-index: 50;
  }
  .brand { font-family: 'Orbitron', sans-serif; font-weight: 900; font-size: 13px; letter-spacing: 1px; color: #fff; display: flex; align-items: center; gap: 6px; }
  .brand span { color: var(--accent-cyan); text-shadow: 0 0 10px rgba(0,240,255,0.6); }
  
  /* Status Badge with Live Radar Glow */
  .conn-pill {
    display: flex; align-items: center; gap: 6px;
    padding: 5px 12px; border-radius: 20px; font-size: 10px; font-family: 'Orbitron', sans-serif; font-weight: 800;
    background: rgba(255, 30, 86, 0.15); border: 1px solid var(--accent-red); color: var(--accent-red);
    transition: all 0.4s ease;
  }
  .conn-pill.online {
    background: rgba(0, 255, 136, 0.15); border-color: var(--accent-green); color: var(--accent-green);
    box-shadow: 0 0 15px rgba(0,255,136,0.4);
  }
  .conn-dot {
    width: 8px; height: 8px; border-radius: 50%; background: currentColor;
    box-shadow: 0 0 10px currentColor; animation: radarPulse 1.5s infinite;
  }
  @keyframes radarPulse { 0% { transform: scale(0.95); opacity: 0.8; box-shadow: 0 0 0 0 rgba(255,30,86, 0.7); } 70% { transform: scale(1); opacity: 1; box-shadow: 0 0 0 8px rgba(255,30,86, 0); } 100% { transform: scale(0.95); opacity: 0.8; box-shadow: 0 0 0 0 rgba(255,30,86, 0); } }
  .conn-pill.online .conn-dot { animation: radarPulseGreen 1.5s infinite; }
  @keyframes radarPulseGreen { 0% { box-shadow: 0 0 0 0 rgba(0,255,136, 0.7); } 70% { box-shadow: 0 0 0 8px rgba(0,255,136, 0); } 100% { box-shadow: 0 0 0 0 rgba(0,255,136, 0); } }

  .container { max-width: 600px; margin: 0 auto; padding: 12px; }

  /* Smooth Fade-in Animation for Tabs */
  .tab-pane { animation: fadeInScale 0.3s ease-in-out; }
  @keyframes fadeInScale { from { opacity: 0; transform: translateY(6px); } to { opacity: 1; transform: translateY(0); } }

  /* Fault Alert */
  .fault-banner {
    background: linear-gradient(90deg, rgba(255,30,86,0.3), rgba(255,30,86,0.1));
    border-left: 4px solid var(--accent-red);
    color: #ff6b8b; padding: 10px 12px; border-radius: 8px; margin-bottom: 12px;
    font-family: 'Orbitron', sans-serif; font-size: 11px; display: none; font-weight: 700;
    animation: pulseAlert 1.2s infinite;
  }
  @keyframes pulseAlert { 50% { opacity: 0.5; } }

  /* Navigation Pills */
  .nav-tabs {
    display: grid; grid-template-columns: repeat(4, 1fr); gap: 6px;
    background: var(--bg-panel); padding: 5px; border-radius: 12px;
    border: 1px solid var(--border-subtle); margin-bottom: 14px;
  }
  .nav-btn {
    background: transparent; border: none; color: var(--text-dim);
    padding: 9px 4px; font-family: 'Orbitron', sans-serif; font-size: 10px; font-weight: 800;
    border-radius: 8px; cursor: pointer; transition: all 0.25s ease; letter-spacing: 0.5px;
  }
  .nav-btn:hover { color: #fff; background: rgba(255,255,255,0.03); }
  .nav-btn.active {
    background: linear-gradient(135deg, rgba(0,240,255,0.25), rgba(0,255,136,0.2));
    color: #fff; border: 1px solid rgba(0,240,255,0.5);
    box-shadow: 0 0 15px rgba(0,240,255,0.2);
  }

  /* Glass Cards */
  .card {
    background: var(--bg-glass);
    backdrop-filter: blur(14px);
    border: 1px solid var(--border-subtle);
    border-radius: 16px; padding: 14px; margin-bottom: 12px;
    box-shadow: 0 10px 30px rgba(0,0,0,0.5);
    transition: transform 0.2s ease;
  }

  /* ARC TACHOMETER CANVAS */
  .tacho-wrapper { position: relative; width: 100%; display: flex; flex-direction: column; align-items: center; justify-content: center; padding: 4px 0; }
  #tachoCanvas { width: 260px; height: 220px; display: block; filter: drop-shadow(0 0 10px rgba(0,240,255,0.15)); }
  .shift-indicator {
    position: absolute; top: 8px; right: 10px;
    font-family: 'Orbitron', sans-serif; font-size: 9px; font-weight: 800;
    padding: 4px 9px; border-radius: 6px; background: #161a24; color: var(--text-dim);
    border: 1px solid var(--border-subtle); transition: all 0.1s;
  }
  .shift-indicator.flash { background: var(--accent-red); color: #fff; box-shadow: 0 0 20px var(--accent-red); border-color: #fff; transform: scale(1.05); }

  /* Metric Grid */
  .metric-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 12px; }
  .metric-box {
    background: var(--bg-panel); border: 1px solid var(--border-subtle);
    border-radius: 12px; padding: 12px; position: relative; overflow: hidden;
  }
  .metric-label { font-size: 11px; color: var(--text-dim); font-weight: 700; letter-spacing: 1px; display: flex; justify-content: space-between; }
  .metric-val { font-family: 'Orbitron', sans-serif; font-size: 22px; font-weight: 800; margin: 6px 0 8px 0; color: #fff; }
  .metric-bar-bg { width: 100%; height: 6px; background: #1a1f2e; border-radius: 4px; overflow: hidden; }
  .metric-bar-fill { height: 100%; width: 0%; border-radius: 4px; transition: width 0.1s linear, background-color 0.2s; }

  /* Chart Box */
  .chart-header { font-family: 'Orbitron', sans-serif; font-size: 10px; color: var(--accent-cyan); letter-spacing: 1px; margin-bottom: 8px; display: flex; justify-content: space-between; }
  canvas.graph { width: 100%; height: 155px; display: block; }

  /* Tuning Controls */
  .controls-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
  label.cfg-lbl { font-family: 'Orbitron', sans-serif; font-size: 9px; color: var(--text-dim); display: block; margin-bottom: 5px; letter-spacing: 1px; }
  select, input.cfg {
    width: 100%; background: #080a0f; color: var(--accent-cyan);
    border: 1px solid rgba(255,255,255,0.12); padding: 10px;
    font-family: 'Orbitron', sans-serif; font-size: 12px; font-weight: 700;
    border-radius: 8px; outline: none; transition: border-color 0.2s;
  }
  select:focus, input.cfg:focus { border-color: var(--accent-cyan); box-shadow: 0 0 10px rgba(0,240,255,0.3); }

  .quick-adj-bar { display: flex; gap: 6px; margin-bottom: 12px; background: var(--bg-panel); padding: 8px; border-radius: 10px; border: 1px solid var(--border-subtle); align-items: center; }
  .btn-adj { flex: 1; background: #161b26; border: 1px solid rgba(0,240,255,0.3); color: var(--accent-cyan); font-family: 'Orbitron', sans-serif; font-weight: 800; font-size: 11px; padding: 8px 0; border-radius: 6px; cursor: pointer; transition: 0.2s; }
  .btn-adj:active, .btn-adj:hover { background: var(--accent-cyan); color: #000; box-shadow: 0 0 10px rgba(0,240,255,0.4); }

  /* Tables */
  .table-wrap { overflow-x: auto; max-height: 340px; border-radius: 10px; border: 1px solid var(--border-subtle); background: #090b10; margin-bottom: 12px; }
  table { width: 100%; border-collapse: collapse; }
  th, td { border: 1px solid rgba(255,255,255,0.04); padding: 7px 4px; text-align: center; font-size: 13px; }
  th { background: #121622; color: var(--accent-cyan); font-family: 'Orbitron', sans-serif; font-size: 10px; position: sticky; top: 0; z-index: 2; }
  
  input.cell {
    width: 44px; background: rgba(255,255,255,0.03); color: #fff;
    border: 1px solid rgba(255,255,255,0.1); text-align: center; padding: 6px 2px;
    font-family: 'Chakra Petch', sans-serif; font-weight: 700; font-size: 14px; border-radius: 6px; outline: none;
    transition: transform 0.15s;
  }
  input.cell:focus { border-color: #fff; box-shadow: 0 0 10px rgba(255,255,255,0.5); transform: scale(1.08); background: rgba(0,240,255,0.15); }
  input.dwell-cell { width: 85px; color: var(--accent-cyan); }

  /* Action Buttons with Live Shimmer Effect */
  .btn-primary, .btn-danger, .btn-secondary {
    position: relative; overflow: hidden; font-family: 'Orbitron', sans-serif; font-weight: 900;
    border: none; padding: 14px; font-size: 12px; border-radius: 10px; cursor: pointer;
    width: 100%; letter-spacing: 1.5px; text-transform: uppercase; margin-bottom: 8px;
    transition: filter 0.2s, transform 0.1s; display: block; text-align: center; text-decoration: none;
  }
  .btn-primary:active, .btn-danger:active, .btn-secondary:active { transform: scale(0.98); }
  
  .btn-primary {
    background: linear-gradient(135deg, #00f0ff, #0077ff); color: #000;
    box-shadow: 0 4px 18px rgba(0, 240, 255, 0.35);
  }
  .btn-danger {
    background: linear-gradient(135deg, #ff1e56, #ff5e00); color: #fff;
    box-shadow: 0 4px 18px rgba(255, 30, 86, 0.35);
  }
  .btn-secondary {
    background: #161b26; color: var(--text-primary); border: 1px solid var(--border-subtle); font-weight: 700; font-size: 11px; padding: 11px;
  }
  
  /* Shimmer Shine Animation on Buttons */
  .btn-primary::after, .btn-danger::after {
    content: ''; position: absolute; top: -50%; left: -50%; width: 200%; height: 200%;
    background: linear-gradient(60deg, transparent, rgba(255,255,255,0.25), transparent);
    transform: rotate(30deg) translateX(-100%); animation: shimmerShine 3s infinite;
  }
  @keyframes shimmerShine { 0% { transform: rotate(30deg) translateX(-100%); } 20% { transform: rotate(30deg) translateX(100%); } 100% { transform: rotate(30deg) translateX(100%); } }

  .tr-alert { background: rgba(255, 30, 86, 0.18) !important; color: #ff6b8b !important; }
</style>
</head>
<body>

<div class="marquee-strip">
  <div class="marquee-inner">
    <div class="marquee-text">🏁 TEAM PATAS KUDUS SATRIA CLUB &bull; ESP32-S3 IGNITION & TELEMETRY LIVE SYSTEM &bull; HIGH PERFORMANCE RACING TCI &bull; 🏁</div>
  </div>
</div>

<div class="top-bar">
  <div class="brand">⚡ PATAS KUDUS <span>S3 TCI</span></div>
  <div class="conn-pill" id="connPill"><div class="conn-dot"></div><span id="ecuStatusText">MENCARI ECU...</span></div>
</div>

<div class="container">
  <div id="faultBanner" class="fault-banner"></div>

  <div class="nav-tabs">
    <button class="nav-btn active" id="btnTabLive" onclick="switchMainTab('live')">COCKPIT</button>
    <button class="nav-btn" id="btnTabDegree" onclick="switchMainTab('degree')">MAP 3D</button>
    <button class="nav-btn" id="btnTabDwell" onclick="switchMainTab('dwell')">DWELL</button>
    <button class="nav-btn" id="btnTabLogger" onclick="switchMainTab('logger')">LOGGER</button>
  </div>

  <!-- 1. COCKPIT / LIVE MONITOR -->
  <div id="liveSection" class="tab-pane">
    <div class="card tacho-wrapper">
      <div class="shift-indicator" id="shiftLight">SHIFT LIMIT</div>
      <canvas id="tachoCanvas" width="520" height="440"></canvas>
    </div>

    <div class="metric-grid">
      <div class="metric-box">
        <div class="metric-label"><span>IGNITION ADV</span><span>MAX 45&deg;</span></div>
        <div class="metric-val" id="valDeg" style="color: var(--accent-green);">0.0&deg;</div>
        <div class="metric-bar-bg"><div class="metric-bar-fill" id="barDeg" style="background: var(--accent-green);"></div></div>
      </div>

      <div class="metric-box">
        <div class="metric-label"><span>THROTTLE (TPS)</span><span>0-100%</span></div>
        <div class="metric-val" id="valTps" style="color: var(--accent-cyan);">0%</div>
        <div class="metric-bar-bg"><div class="metric-bar-fill" id="barTps" style="background: var(--accent-cyan);"></div></div>
      </div>

      <div class="metric-box">
        <div class="metric-label"><span>ENGINE TEMP</span><span>NTC</span></div>
        <div class="metric-val" id="valTemp" style="color: var(--accent-amber);">0.0&deg;C</div>
        <div class="metric-bar-bg"><div class="metric-bar-fill" id="barTemp" style="background: var(--accent-amber);"></div></div>
      </div>

      <div class="metric-box">
        <div class="metric-label"><span>BATTERY</span><span>VOLT</span></div>
        <div class="metric-val" id="valBatt" style="color: #c77dff;">0.0V</div>
        <div class="metric-bar-bg"><div class="metric-bar-fill" id="barBatt" style="background: #c77dff;"></div></div>
      </div>
    </div>

    <div class="card">
      <div class="chart-header">
        <span>LIVE TELEMETRY STREAM</span>
        <span><span style="color:var(--accent-green);">&bull; RPM</span> &nbsp; <span style="color:var(--accent-cyan);">&bull; ADV&deg;</span></span>
      </div>
      <canvas id="realtimeChart" class="graph"></canvas>
    </div>
  </div>

  <!-- 2. TUNING SECTION -->
  <div id="tuningSection" class="tab-pane" style="display:none;">
    <div class="card">
      <div class="controls-grid">
        <div>
          <label class="cfg-lbl">PILIH SLOT MAP</label>
          <select id="slotSelect" onchange="loadMap()">
            <option value="0">SLOT 1 — DAILY / TOURING</option>
            <option value="1">SLOT 2 — SPORT / KOPDAR</option>
            <option value="2">SLOT 3 — RACING 150CC</option>
            <option value="3">SLOT 4 — DRAG / EXTREME</option>
            <option value="4">SLOT 5 — CUSTOM USER</option>
          </select>
        </div>
        <div>
          <label class="cfg-lbl">RPM LIMITER (HARD CUT)</label>
          <input type="number" id="rpmLimit" class="cfg" value="12500" step="100">
        </div>
      </div>
    </div>

    <div id="degTools">
      <div class="quick-adj-bar">
        <span style="font-size:10px; color:var(--text-dim); font-family:'Orbitron'; padding:0 4px;">KOREKSI:</span>
        <button class="btn-adj" onclick="adjustSelectedCell(-10)">-1.0&deg;</button>
        <button class="btn-adj" onclick="adjustSelectedCell(-5)">-0.5&deg;</button>
        <button class="btn-adj" onclick="adjustSelectedCell(5)">+0.5&deg;</button>
        <button class="btn-adj" onclick="adjustSelectedCell(10)">+1.0&deg;</button>
      </div>
    </div>

    <div class="card">
      <div class="chart-header"><span id="chartTitle">KURVA PENGAPIAN (DEGREE VS RPM)</span></div>
      <canvas id="tuningChart" class="graph"></canvas>
    </div>

    <div id="tabDegree" class="table-wrap">
      <table>
        <thead><tr><th>RPM \ TPS</th><th>0%</th><th>25%</th><th>50%</th><th>75%</th><th>100%</th></tr></thead>
        <tbody id="mapBody"></tbody>
      </table>
    </div>

    <div id="tabDwell" class="table-wrap" style="display:none;">
      <table>
        <thead><tr><th>RPM AXIS</th><th>DWELL (&mu;s)</th><th>KARAKTER</th></tr></thead>
        <tbody id="dwellBody"></tbody>
      </table>
    </div>

    <button class="btn-danger" onclick="saveAndSend()">💾 SIMPAN & FLASH KE ECU S3</button>
  </div>

  <!-- 3. DATA LOGGER -->
  <div id="loggerSection" class="tab-pane" style="display:none;">
    <div class="card" style="text-align:center;">
      <div id="overflowBadge" style="display:none; color:var(--accent-amber); font-size:11px; margin-bottom:10px; font-family:'Orbitron';">⚠️ BUFFER PENUH — LOG TERLAMA TERTIMPA</div>
      <button class="btn-primary" onclick="fetchLiveDataLog()">🔄 REFRESH DATA LOG</button>
      <a href="/downloadCSV" class="btn-secondary">📥 DOWNLOAD CSV (EXCEL)</a>
      <button class="btn-secondary" onclick="clearLogData()" style="color:var(--accent-red); border-color:rgba(255,30,86,0.3);">🗑️ HAPUS MEMORI LOG</button>
    </div>

    <div class="table-wrap">
      <table>
        <thead><tr><th>WAKTU</th><th>RPM</th><th>TPS</th><th>ADV</th><th>SUHU</th><th>AKI</th></tr></thead>
        <tbody id="logTableBody">
          <tr><td colspan="6" style="color:var(--text-dim); padding:20px;">Klik Refresh Data Log untuk memuat rekaman.</td></tr>
        </tbody>
      </table>
    </div>
  </div>
</div>

<script>
const rpmLabels = ["0k","1k","2k","3k","4k","5k","6k","7k","8k","9k","10k","11k","12k","13k","14k"];
let lastFocusedCell = null;

let targetData  = { rpm: 0, deg: 0, tps: 0, temp: 0, batt: 12.6, limit: 12500 };
let displayData = { rpm: 0, deg: 0, tps: 0, temp: 0, batt: 12.6 };

let rtHistoryRpm = new Array(80).fill(0);
let rtHistoryDeg = new Array(80).fill(0);
let lastGraphPush = 0;

function lerp(start, end, amt) {
  return (1 - amt) * start + amt * end;
}

function renderLoop(timestamp) {
  displayData.rpm  = lerp(displayData.rpm,  targetData.rpm,  0.22);
  displayData.deg  = lerp(displayData.deg,  targetData.deg,  0.25);
  displayData.tps  = lerp(displayData.tps,  targetData.tps,  0.28);
  displayData.temp = lerp(displayData.temp, targetData.temp, 0.15);
  displayData.batt = lerp(displayData.batt, targetData.batt, 0.15);

  if (Math.abs(targetData.rpm - displayData.rpm) < 1) displayData.rpm = targetData.rpm;

  if (document.getElementById('liveSection').style.display !== 'none') {
    drawTachometer(displayData.rpm, targetData.limit);

    document.getElementById('valDeg').innerText  = displayData.deg.toFixed(1) + '°';
    document.getElementById('barDeg').style.width = Math.min(Math.max((displayData.deg / 45.0) * 100, 0), 100) + '%';

    document.getElementById('valTps').innerText  = Math.round(displayData.tps) + '%';
    document.getElementById('barTps').style.width = Math.min(Math.max(displayData.tps, 0), 100) + '%';

    document.getElementById('valTemp').innerText = displayData.temp.toFixed(1) + '°C';
    document.getElementById('barTemp').style.width = Math.min(Math.max((displayData.temp / 120.0) * 100, 0), 100) + '%';

    document.getElementById('valBatt').innerText = displayData.batt.toFixed(1) + 'V';
    let battPct = ((displayData.batt - 9.0) / 7.0) * 100;
    document.getElementById('barBatt').style.width = Math.min(Math.max(battPct, 0), 100) + '%';

    if (timestamp - lastGraphPush > 50) {
      rtHistoryRpm.push(displayData.rpm);
      rtHistoryDeg.push(displayData.deg);
      rtHistoryRpm.shift();
      rtHistoryDeg.shift();
      drawRealtimeChart();
      lastGraphPush = timestamp;
    }
  }
  requestAnimationFrame(renderLoop);
}
requestAnimationFrame(renderLoop);

function drawTachometer(rpm, limit) {
  const canvas = document.getElementById('tachoCanvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  const cx = w / 2, cy = h / 2 + 25;
  const radius = 175;
  const maxRpm = 15000;

  ctx.clearRect(0, 0, w, h);

  const startAngle = 0.75 * Math.PI;
  const endAngle   = 2.25 * Math.PI;
  const totalSweep = endAngle - startAngle;

  ctx.beginPath();
  ctx.arc(cx, cy, radius, startAngle, endAngle);
  ctx.lineWidth = 26;
  ctx.strokeStyle = '#151924';
  ctx.lineCap = 'round';
  ctx.stroke();

  let ratio = Math.min(Math.max(rpm / maxRpm, 0), 1);
  let activeAngle = startAngle + ratio * totalSweep;
  let isShift = rpm >= (limit - 400) && rpm > 2000;

  let shiftEl = document.getElementById('shiftLight');
  if (isShift) shiftEl.classList.add('flash');
  else shiftEl.classList.remove('flash');

  let grad = ctx.createLinearGradient(cx - radius, cy, cx + radius, cy);
  if (isShift) {
    grad.addColorStop(0, '#ff1e56');
    grad.addColorStop(1, '#ff7b00');
  } else {
    grad.addColorStop(0, '#00f0ff');
    grad.addColorStop(0.6, '#00ff88');
    grad.addColorStop(1, '#ffb800');
  }

  if (rpm > 10) {
    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, radius, startAngle, activeAngle);
    ctx.lineWidth = 26;
    ctx.strokeStyle = grad;
    ctx.lineCap = 'round';
    ctx.shadowColor = isShift ? '#ff1e56' : '#00f0ff';
    ctx.shadowBlur = 22;
    ctx.stroke();
    ctx.restore();
  }

  ctx.font = '700 18px Orbitron, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  for (let i = 0; i <= 15; i++) {
    let a = startAngle + (i / 15) * totalSweep;
    let tx = cx + Math.cos(a) * (radius - 38);
    let ty = cy + Math.sin(a) * (radius - 38);
    ctx.fillStyle = (i >= 12) ? '#ff1e56' : '#6c768f';
    ctx.fillText(i.toString(), tx, ty);
  }

  ctx.fillStyle = isShift ? '#ff1e56' : '#ffffff';
  ctx.font = '900 64px Orbitron, sans-serif';
  ctx.fillText(Math.round(rpm).toString(), cx, cy - 8);

  ctx.fillStyle = '#00f0ff';
  ctx.font = '700 18px Orbitron, sans-serif';
  ctx.fillText('RPM', cx, cy + 36);
}

function pollLiveStatus() {
  fetch('/getLiveStatus').then(r => r.json()).then(data => {
    let pill = document.getElementById('connPill');
    let statusTxt = document.getElementById('ecuStatusText');
    let faultEl = document.getElementById('faultBanner');

    if (data.ecuOnline) {
      pill.classList.add('online');
      statusTxt.innerText = "ECU TERHUBUNG";
    } else {
      pill.classList.remove('online');
      statusTxt.innerText = "MENCARI ECU...";
    }

    targetData.rpm  = data.rpm || 0;
    targetData.deg  = (data.deg || 0) / 10.0;
    targetData.tps  = data.tps || 0;
    targetData.temp = (data.temp || 0) / 10.0;
    targetData.batt = (data.batt || 0) / 10.0;
    targetData.limit = parseInt(document.getElementById('rpmLimit').value) || 12500;

    if (data.faultCode && data.faultCode !== 0) {
      let msgs = [];
      if (data.faultCode & 1) msgs.push("TPS SENSOR");
      if (data.faultCode & 2) msgs.push("NTC TEMP");
      if (data.faultCode & 4) msgs.push("LOW BATT (<10V)");
      if (data.faultCode & 8) msgs.push("OVERCHARGE (>16V)");
      faultEl.innerText = "⚠️ KENDALA: " + msgs.join(" • ");
      faultEl.style.display = "block";
    } else {
      faultEl.style.display = "none";
    }
  }).catch(() => {});
}
setInterval(pollLiveStatus, 100);

function drawRealtimeChart() {
  const canvas = document.getElementById('realtimeChart');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  canvas.width = canvas.clientWidth * 2;
  canvas.height = canvas.clientHeight * 2;
  const w = canvas.width, h = canvas.height;

  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = 'rgba(255,255,255,0.05)'; ctx.lineWidth = 2;
  for (let y = 0; y <= h; y += h / 4) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }

  let stepX = w / (rtHistoryRpm.length - 1);

  ctx.strokeStyle = '#00ff88'; ctx.lineWidth = 4; ctx.beginPath();
  for (let i = 0; i < rtHistoryRpm.length; i++) {
    let x = i * stepX;
    let y = h - ((rtHistoryRpm[i] / 15000) * h);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.stroke();

  ctx.strokeStyle = '#00f0ff'; ctx.lineWidth = 3; ctx.beginPath();
  for (let i = 0; i < rtHistoryDeg.length; i++) {
    let x = i * stepX;
    let y = h - ((rtHistoryDeg[i] / 45.0) * h);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.stroke();
}

function switchMainTab(tab) {
  document.getElementById('liveSection').style.display    = (tab === 'live') ? 'block' : 'none';
  document.getElementById('tuningSection').style.display  = (tab === 'degree' || tab === 'dwell') ? 'block' : 'none';
  document.getElementById('loggerSection').style.display  = (tab === 'logger') ? 'block' : 'none';

  ['btnTabLive', 'btnTabDegree', 'btnTabDwell', 'btnTabLogger'].forEach(id => document.getElementById(id).classList.remove('active'));

  if (tab === 'live') {
    document.getElementById('btnTabLive').classList.add('active');
  } else if (tab === 'degree') {
    document.getElementById('btnTabDegree').classList.add('active');
    document.getElementById('tabDegree').style.display = 'block';
    document.getElementById('tabDwell').style.display  = 'none';
    document.getElementById('degTools').style.display  = 'block';
    document.getElementById('chartTitle').innerText    = "KURVA PENGAPIAN (DEGREE VS RPM)";
    drawChart();
  } else if (tab === 'dwell') {
    document.getElementById('btnTabDwell').classList.add('active');
    document.getElementById('tabDegree').style.display = 'none';
    document.getElementById('tabDwell').style.display  = 'block';
    document.getElementById('degTools').style.display  = 'none';
    document.getElementById('chartTitle').innerText    = "KURVA DWELL TIME (MICROSECONDS)";
    drawChart();
  } else if (tab === 'logger') {
    document.getElementById('btnTabLogger').classList.add('active');
    fetchLiveDataLog();
  }
}

function applyHeatmapColors() {
  for (let r = 0; r < 15; r++) {
    for (let t = 0; t < 5; t++) {
      let cell = document.getElementById(`c_${r}_${t}`);
      if (!cell) continue;
      let val = parseInt(cell.value) || 0;
      let ratio = Math.min(Math.max((val - 100) / 280, 0), 1);
      let hue = (1 - ratio) * 200;
      cell.style.backgroundColor = `hsla(${hue}, 85%, 45%, 0.25)`;
      cell.style.borderColor     = `hsla(${hue}, 90%, 55%, 0.6)`;
      cell.style.color           = `hsl(${hue}, 100%, 75%)`;
    }
  }
}

function adjustSelectedCell(delta) {
  if (!lastFocusedCell) { alert('Ketuk salah satu kotak angka pada tabel Map 3D terlebih dahulu.'); return; }
  let val = (parseInt(lastFocusedCell.value) || 0) + delta;
  lastFocusedCell.value = Math.min(Math.max(val, 0), 450);
  drawChart();
  applyHeatmapColors();
}

function drawChart() {
  const canvas = document.getElementById('tuningChart');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  canvas.width = canvas.clientWidth * 2;
  canvas.height = canvas.clientHeight * 2;
  const w = canvas.width, h = canvas.height;
  const padL = 55, padR = 25, padT = 25, padB = 45;
  const plotW = w - padL - padR, plotH = h - padT - padB;

  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = 'rgba(255,255,255,0.06)'; ctx.lineWidth = 2;
  ctx.fillStyle = '#7b849b'; ctx.font = '700 18px Orbitron, sans-serif';

  let isDeg = document.getElementById('tabDegree').style.display !== 'none';
  if (isDeg) {
    let minY = 0, maxY = 450;
    for (let v = 0; v <= 400; v += 100) {
      let y = padT + plotH - ((v - minY) / (maxY - minY)) * plotH;
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
      ctx.fillText((v / 10) + '°', 8, y + 6);
    }
    for (let r = 0; r < 15; r += 2) {
      let x = padL + (r / 14) * plotW;
      ctx.fillText(rpmLabels[r], x - 14, h - 12);
    }
    const colors = ['#00f0ff', '#00ff88', '#ffb800', '#ff7b00', '#ff1e56'];
    for (let t = 0; t < 5; t++) {
      ctx.strokeStyle = colors[t]; ctx.lineWidth = 4; ctx.beginPath();
      for (let r = 0; r < 15; r++) {
        let el = document.getElementById(`c_${r}_${t}`);
        let val = el ? (parseInt(el.value) || 0) : 100;
        let x = padL + (r / 14) * plotW;
        let y = padT + plotH - ((val - minY) / (maxY - minY)) * plotH;
        if (r === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
  } else {
    let minY = 1000, maxY = 5000;
    for (let v = 1000; v <= 5000; v += 1000) {
      let y = padT + plotH - ((v - minY) / (maxY - minY)) * plotH;
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
      ctx.fillText((v / 1000) + 'k', 8, y + 6);
    }
    for (let r = 0; r < 15; r += 2) {
      let x = padL + (r / 14) * plotW;
      ctx.fillText(rpmLabels[r], x - 14, h - 12);
    }
    ctx.strokeStyle = '#00f0ff'; ctx.lineWidth = 5; ctx.beginPath();
    for (let r = 0; r < 15; r++) {
      let el = document.getElementById(`d_${r}`);
      let val = el ? (parseInt(el.value) || 3200) : 3200;
      let x = padL + (r / 14) * plotW;
      let y = padT + plotH - ((val - minY) / (maxY - minY)) * plotH;
      if (r === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
}

function loadMap() {
  let slot = document.getElementById('slotSelect').value;
  fetch('/getMap?slot=' + slot).then(r => r.json()).then(data => {
    document.getElementById('rpmLimit').value = data.rpmLimit;
    let tbodyMap = document.getElementById('mapBody'); tbodyMap.innerHTML = '';
    for (let r = 0; r < 15; r++) {
      let tr = document.createElement('tr');
      let tdRpm = document.createElement('td');
      tdRpm.style.color = '#00f0ff'; tdRpm.style.fontWeight = '700'; tdRpm.innerText = rpmLabels[r];
      tr.appendChild(tdRpm);
      for (let t = 0; t < 5; t++) {
        let td = document.createElement('td');
        let val = (data.map && data.map[r]) ? data.map[r][t] : 100;
        td.innerHTML = `<input type="number" class="cell" id="c_${r}_${t}" value="${val}" onfocus="lastFocusedCell=this" oninput="drawChart(); applyHeatmapColors();">`;
        tr.appendChild(td);
      }
      tbodyMap.appendChild(tr);
    }

    let tbodyDwell = document.getElementById('dwellBody'); tbodyDwell.innerHTML = '';
    for (let r = 0; r < 15; r++) {
      let tr = document.createElement('tr');
      let tdRpm = document.createElement('td'); tdRpm.style.color = '#00f0ff'; tdRpm.innerText = rpmLabels[r] + ' RPM';
      let tdDwell = document.createElement('td');
      let dVal = (data.dwell && data.dwell[r]) ? data.dwell[r] : 3200;
      tdDwell.innerHTML = `<input type="number" class="cell dwell-cell" id="d_${r}" value="${dVal}" step="50" oninput="drawChart()">`;
      let tdDesc = document.createElement('td'); tdDesc.style.color = '#7b849b'; tdDesc.style.fontSize = '11px';
      tdDesc.innerText = r < 4 ? 'Low Charge' : (r < 10 ? 'Mid Optimal' : 'High Speed');
      tr.appendChild(tdRpm); tr.appendChild(tdDwell); tr.appendChild(tdDesc);
      tbodyDwell.appendChild(tr);
    }
    drawChart();
    applyHeatmapColors();
  });
}

function saveAndSend() {
  let slot = parseInt(document.getElementById('slotSelect').value);
  let payload = { slot: slot, rpmLimit: parseInt(document.getElementById('rpmLimit').value) || 12500, map: [], dwell: [] };
  for (let r = 0; r < 15; r++) {
    let row = [];
    for (let t = 0; t < 5; t++) row.push(parseInt(document.getElementById(`c_${r}_${t}`).value) || 100);
    payload.map.push(row);
  }
  for (let r = 0; r < 15; r++) payload.dwell.push(parseInt(document.getElementById(`d_${r}`).value) || 3200);

  fetch('/saveTuning', { method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(payload) })
  .then(() => alert('✅ Slot ' + (slot + 1) + ' Berhasil Tersimpan & Aktif di ECU S3!'));
}

function fetchLiveDataLog() {
  fetch('/getLogData').then(r => r.json()).then(data => {
    let tbody = document.getElementById('logTableBody');
    tbody.innerHTML = '';
    document.getElementById('overflowBadge').style.display = data.overflow ? 'block' : 'none';
    if (!data.logs || data.logs.length === 0) {
      tbody.innerHTML = '<tr><td colspan="6" style="color:var(--text-dim); padding:16px;">Belum ada data terekam.</td></tr>';
      return;
    }
    data.logs.reverse().forEach(e => {
      let tr = document.createElement('tr');
      if ((e.temp / 10.0) >= 95.0 || (e.batt / 10.0) <= 11.2) tr.classList.add('tr-alert');
      tr.innerHTML = `
        <td style="color:var(--text-dim);">${(e.t / 1000).toFixed(1)}s</td>
        <td style="color:var(--accent-green); font-weight:700;">${e.r}</td>
        <td>${e.p}%</td>
        <td style="color:var(--accent-cyan);">${(e.a / 10).toFixed(1)}°</td>
        <td>${(e.temp / 10).toFixed(1)}°</td>
        <td>${(e.batt / 10).toFixed(1)}V</td>`;
      tbody.appendChild(tr);
    });
  }).catch(() => {});
}

function clearLogData() {
  if (confirm('Hapus seluruh riwayat log di RAM?')) {
    fetch('/clearLog').then(() => fetchLiveDataLog());
  }
}

window.onload = function() {
  fetch('/getLiveStatus').then(r => r.json()).then(d => {
    document.getElementById('slotSelect').value = d.activeSlot;
    loadMap();
  }).catch(() => loadMap());
};
window.onresize = function() { drawChart(); };
</script>
</body>
</html>
)rawliteral";

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ 
    lastActivityTime = millis();
    request->send_P(200, "text/html", htmlUI); 
  });
  
  server.on("/getLiveStatus", HTTP_GET, [](AsyncWebServerRequest *request){
    lastActivityTime = millis();
    JsonDocument doc;
    
    portENTER_CRITICAL(&dataMux);
    doc["rpm"] = currentTelemetry.rpm;
    doc["tps"] = currentTelemetry.tps;
    doc["deg"] = currentTelemetry.degree10;
    doc["temp"] = currentTelemetry.temp10;
    doc["batt"] = currentTelemetry.battVolt10;
    doc["faultCode"] = currentTelemetry.faultCode;
    doc["ackStatus"] = currentTelemetry.ackStatus;
    portEXIT_CRITICAL(&dataMux);

    doc["ecuOnline"] = isEcuConnected;
    doc["activeSlot"] = currentActiveSlot;

    String resp;
    serializeJson(doc, resp);
    request->send(200, "application/json", resp);
  });

  server.on("/getMap", HTTP_GET, [](AsyncWebServerRequest *request){
    lastActivityTime = millis();
    uint8_t slot = request->hasParam("slot") ? request->getParam("slot")->value().toInt() : 0;
    if (slot > 4) slot = 0;
    JsonDocument doc;
    doc["rpmLimit"] = customMapSlots[slot].rpmLimit;
    JsonArray arrMap = doc["map"].to<JsonArray>();
    for (int r = 0; r < NUM_RPM_POINTS; r++) {
      JsonArray row = arrMap.add<JsonArray>();
      for (int t = 0; t < NUM_TPS_POINTS; t++) row.add(customMapSlots[slot].mapData[r][t]);
    }
    JsonArray arrDwell = doc["dwell"].to<JsonArray>();
    for (int r = 0; r < NUM_RPM_POINTS; r++) arrDwell.add(customMapSlots[slot].dwellData[r]);
    String response; serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  server.on("/getLogData", HTTP_GET, [](AsyncWebServerRequest *request){
    lastActivityTime = millis();
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    JsonDocument doc;
    doc["overflow"] = bufferOverflowed;
    
    JsonArray arr = doc["logs"].to<JsonArray>();
    portENTER_CRITICAL(&logMux);
    uint16_t startIdx = (logCount < MAX_LOG_ENTRIES) ? 0 : logHead;
    for (uint16_t i = 0; i < logCount; i++) {
      uint16_t idx = (startIdx + i) % MAX_LOG_ENTRIES;
      JsonObject entry = arr.add<JsonObject>();
      entry["t"] = logBuffer[idx].timestampMs;
      entry["r"] = logBuffer[idx].rpm;
      entry["p"] = logBuffer[idx].tps;
      entry["a"] = logBuffer[idx].degree10;
      entry["temp"] = logBuffer[idx].temp10;
      entry["batt"] = logBuffer[idx].battVolt10;
    }
    portEXIT_CRITICAL(&logMux);
    serializeJson(doc, *response);
    request->send(response);
  });

  server.on("/clearLog", HTTP_GET, [](AsyncWebServerRequest *request){
    lastActivityTime = millis();
    portENTER_CRITICAL(&logMux);
    logHead = 0; logCount = 0; bufferOverflowed = false;
    portEXIT_CRITICAL(&logMux);
    request->send(200, "text/plain", "Log Cleared");
  });

  server.on("/downloadCSV", HTTP_GET, [](AsyncWebServerRequest *request){
    lastActivityTime = millis();
    AsyncResponseStream *response = request->beginResponseStream("text/csv");
    response->addHeader("Content-Disposition", "attachment; filename=satria_ecu_log.csv");
    response->print("Timestamp_ms,RPM,TPS_Pct,Adv_Deg,Temp_C,Batt_V\n");
    portENTER_CRITICAL(&logMux);
    uint16_t startIdx = (logCount < MAX_LOG_ENTRIES) ? 0 : logHead;
    for (uint16_t i = 0; i < logCount; i++) {
      uint16_t idx = (startIdx + i) % MAX_LOG_ENTRIES;
      response->printf("%lu,%u,%u,%.1f,%.1f,%.1f\n",
                       logBuffer[idx].timestampMs, logBuffer[idx].rpm, logBuffer[idx].tps,
                       (float)logBuffer[idx].degree10 / 10.0f,
                       (float)logBuffer[idx].temp10 / 10.0f,
                       (float)logBuffer[idx].battVolt10 / 10.0f);
    }
    portEXIT_CRITICAL(&logMux);
    request->send(response);
  });

  AsyncCallbackJsonWebHandler* handler = new AsyncCallbackJsonWebHandler("/saveTuning", [](AsyncWebServerRequest *request, JsonVariant &json) {
    lastActivityTime = millis();
    JsonObject obj = json.as<JsonObject>();
    uint8_t slot = obj["slot"].as<uint8_t>();
    if (slot > 4) slot = 0;
    
    currentActiveSlot = slot;
    customMapSlots[slot].rpmLimit = obj["rpmLimit"].as<uint16_t>();
    
    JsonArray arrMap = obj["map"].as<JsonArray>(); int r = 0;
    for(JsonVariant row : arrMap) {
      if(r >= NUM_RPM_POINTS) break; int t = 0;
      for(JsonVariant val : row.as<JsonArray>()) { 
        if(t >= NUM_TPS_POINTS) break; 
        customMapSlots[slot].mapData[r][t] = val.as<int16_t>(); t++; 
      } r++;
    }
    JsonArray arrDwell = obj["dwell"].as<JsonArray>(); int d = 0;
    for(JsonVariant val : arrDwell) {
      if(d >= NUM_RPM_POINTS) break;
      customMapSlots[slot].dwellData[d] = val.as<uint16_t>();
      d++;
    }
    saveMapToNVS();
    sendCommandToECU(0x03, slot); 
    request->send(200, "text/plain", "Saved & Sent");
    lastActivityTime = millis();
  });
  
  server.addHandler(handler);
  server.begin();
}

void setup() {
  setCpuFrequencyMhz(80); 
  Serial1.begin(250000, SERIAL_8N1, RX_PIN, TX_PIN);
  
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("ECU_SATRIA_FU", "kudus1234", 1); 
  esp_wifi_set_max_tx_power(50);
  
  esp_wifi_set_mac(WIFI_IF_STA, transmitterMac);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(OnEspNowSent);
    esp_now_register_recv_cb(OnEspNowRecv);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, displayMacAddress, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }

  loadMapFromNVS();
  setupWebServer();
  
  lastActivityTime = millis();
}

void loop() {
  parseUART();
  
  if (isEcuConnected && !initialSyncDone) {
    if (millis() - lastSyncRetryTime > 500) {
      sendCommandToECU(0x02, currentActiveSlot);
      lastSyncRetryTime = millis();
      syncRetryCount++;
      
      if (syncRetryCount >= 10) {
        initialSyncDone = true;
      }
    }
  }

  if (millis() - lastUartRxTime > UART_TIMEOUT_MS) {
    isEcuConnected = false;
    initialSyncDone = false;
    syncRetryCount = 0;
    portENTER_CRITICAL(&dataMux);
    currentTelemetry.rpm = 0;
    portEXIT_CRITICAL(&dataMux);
  }

  if (isWifiActive && (millis() - lastActivityTime > WIFI_TIMEOUT_MS)) {
    server.end();                 
    WiFi.softAPdisconnect(true);  
    WiFi.mode(WIFI_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    isWifiActive = false;         
  }
  
  delay(1);
}
