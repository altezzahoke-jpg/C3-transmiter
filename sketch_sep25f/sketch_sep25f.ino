#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <Preferences.h>

#define RX_PIN 20
#define TX_PIN 21

// --- MAC ADDRESS TRANSMITTER ---
uint8_t transmitterMac[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC}; 
uint8_t displayMacAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

#define NUM_RPM_POINTS 15
#define NUM_TPS_POINTS 5
Preferences preferences;
AsyncWebServer server(80);

// --- VARIABEL EFISIENSI & TIMEOUT WIFI ---
unsigned long lastActivityTime = 0;
const unsigned long WIFI_TIMEOUT_MS = 180000; // 3 Menit (180.000 ms)
bool isWifiActive = true;

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

// --- STRUKTUR DATA TUNING ---
struct CustomTuning {
  int16_t mapData[NUM_RPM_POINTS][NUM_TPS_POINTS];
  uint16_t dwellData[NUM_RPM_POINTS]; 
  uint16_t rpmLimit;
};
CustomTuning customMapSlots[5];

// --- STRUKTUR DATA TELEMETRI & KOMANDO ---
struct __attribute__((packed)) TelemetryData {
  uint16_t header;     
  uint16_t rpm;
  uint8_t  tps;        
  int16_t  degree10;   
  int16_t  temp10;     
  uint16_t battVolt10; 
  uint8_t  mode;
  uint8_t  fuel;
  uint8_t  state;
  uint16_t crc16;
};

struct __attribute__((packed)) CommandData {
  uint16_t header;     
  uint8_t requestedMode;
  uint16_t crc16;
};

struct __attribute__((packed)) CommandPacketToS3 {
  uint16_t header;       
  uint8_t  cmdType;      
  uint8_t  slotOrMode;   
  int16_t  mapData[NUM_RPM_POINTS][NUM_TPS_POINTS]; 
  uint16_t dwellData[NUM_RPM_POINTS]; 
  uint16_t rpmLimit;     
  uint16_t crc16;      
};

portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
TelemetryData currentTelemetry = {0xAA55, 0, 0, 100, 300, 126, 0, 0, 0, 0};

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

void sendCommandToS3(uint8_t cmdType, uint8_t slot) {
  CommandPacketToS3 cmd;
  cmd.header = 0x55CC;
  cmd.cmdType = cmdType;
  cmd.slotOrMode = slot;
  
  if (cmdType == 0x02 && slot < 5) {
    memcpy(cmd.mapData, customMapSlots[slot].mapData, sizeof(cmd.mapData));
    memcpy(cmd.dwellData, customMapSlots[slot].dwellData, sizeof(cmd.dwellData));
    cmd.rpmLimit = customMapSlots[slot].rpmLimit;
  } else {
    memset(cmd.mapData, 0, sizeof(cmd.mapData)); 
    memcpy(cmd.dwellData, defaultDwellPerRPM, sizeof(cmd.dwellData));
    cmd.rpmLimit = 12500;
  }
  
  cmd.crc16 = calculateCRC16((uint8_t*)&cmd, sizeof(CommandPacketToS3) - sizeof(uint16_t));
  Serial1.write((uint8_t*)&cmd, sizeof(CommandPacketToS3));
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
        esp_now_send(displayMacAddress, rxBuffer, sizeof(TelemetryData));
      }
      rxIndex = 0;
    }
  }
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnEspNowRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len) {
#else
void OnEspNowRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif
  if (len == sizeof(CommandData)) {
    CommandData *cmd = (CommandData*)incomingData;
    if (cmd->header == 0xCC55 && cmd->crc16 == calculateCRC16(incomingData, sizeof(CommandData) - sizeof(uint16_t))) {
      sendCommandToS3(0x01, cmd->requestedMode); 
    }
  }
}

void loadMapFromNVS() {
  preferences.begin("ecu_tuning", false);
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
  preferences.putBytes("customMaps", &customMapSlots, sizeof(customMapSlots));
  preferences.end();
}

// --- TAMPILAN WEB UI RACING THEME LENGKAP ---
const char* htmlUI = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>TEAM PATAS KUDUS - ECU TUNER</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@500;700;900&family=Rajdhani:wght@600;700&display=swap');
  
  body {
    font-family: 'Rajdhani', sans-serif;
    background: #0a0a0c;
    background-image: 
      radial-gradient(circle at 50% 0%, rgba(0, 255, 136, 0.15), transparent 70%),
      linear-gradient(45deg, #111 25%, transparent 25%), 
      linear-gradient(-45deg, #111 25%, transparent 25%),
      linear-gradient(45deg, transparent 75%, #111 75%),
      linear-gradient(-45deg, transparent 75%, #111 75%);
    background-size: 100% 100%, 16px 16px, 16px 16px, 16px 16px, 16px 16px;
    color: #fff;
    margin: 0;
    padding: 0;
  }

  .marquee-box {
    background: linear-gradient(90deg, #ff0055, #00ff87, #00e5ff, #ff0055);
    background-size: 300% 300%;
    animation: gradientMove 6s ease infinite;
    padding: 3px 0;
    box-shadow: 0 0 15px rgba(0, 255, 136, 0.5);
  }
  .marquee-inner {
    background: #000;
    overflow: hidden;
    white-space: nowrap;
    padding: 6px 0;
  }
  .marquee-text {
    display: inline-block;
    font-family: 'Orbitron', sans-serif;
    font-size: 13px;
    font-weight: 700;
    color: #00ff87;
    letter-spacing: 2px;
    text-transform: uppercase;
    animation: marquee 16s linear infinite;
  }

  @keyframes marquee { 0% { transform: translateX(100%); } 100% { transform: translateX(-100%); } }
  @keyframes gradientMove { 0%{background-position:0% 50%} 50%{background-position:100% 50%} 100%{background-position:0% 50%} }

  .header { text-align: center; padding: 15px 10px 5px 10px; }
  .header h1 {
    font-family: 'Orbitron', sans-serif; font-size: 24px; margin: 0;
    color: #fff; text-shadow: 0 0 10px #00ff87, 0 0 20px #00ff87; letter-spacing: 2px;
  }
  
  .timeout-warning {
    text-align: center; font-family: 'Rajdhani', sans-serif; font-size: 14px;
    color: #ff9100; margin-bottom: 15px; font-weight: 700; letter-spacing: 1px;
  }

  .container { max-width: 650px; margin: 0 auto; padding: 10px; }

  .card {
    background: rgba(20, 20, 25, 0.85);
    border: 1px solid rgba(0, 255, 136, 0.3); border-radius: 10px;
    padding: 12px; margin-bottom: 12px;
    box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.7); backdrop-filter: blur(8px);
  }

  .controls-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
  label { font-family: 'Orbitron', sans-serif; font-size: 11px; color: #888; display: block; margin-bottom: 5px; letter-spacing: 1px; }

  select, input[type=number].cfg {
    width: 100%; box-sizing: border-box; background: #050508; color: #00ff87;
    border: 1px solid #222; padding: 8px; font-family: 'Orbitron', sans-serif;
    font-size: 13px; border-radius: 6px; box-shadow: inset 0 0 5px rgba(0, 255, 136, 0.2); outline: none;
  }

  .tab-buttons { display: flex; gap: 8px; margin-bottom: 12px; }
  .tab-btn {
    flex: 1; background: #111; border: 1px solid #333; color: #aaa;
    padding: 10px; font-family: 'Orbitron', sans-serif; font-size: 11px;
    font-weight: 700; border-radius: 6px; cursor: pointer; transition: 0.3s;
  }
  .tab-btn.active {
    background: linear-gradient(180deg, #00ff87, #009951); color: #000;
    border-color: #00ff87; box-shadow: 0 0 12px rgba(0, 255, 136, 0.6);
  }

  .chart-card {
    background: #050508; border: 1px solid #222; border-radius: 8px;
    padding: 10px; margin-bottom: 12px; box-shadow: inset 0 0 10px rgba(0,0,0,0.8);
  }
  .chart-title { font-family: 'Orbitron', sans-serif; font-size: 11px; color: #00e5ff; text-align: center; margin-bottom: 6px; letter-spacing: 1px; }

  .table-responsive { overflow-x: auto; margin-bottom: 15px; border-radius: 8px; border: 1px solid #222; }
  table { width: 100%; border-collapse: collapse; background: #08080c; }
  th, td { border: 1px solid #1a1a24; padding: 5px 2px; text-align: center; font-size: 12px; }
  th { background: #12121a; color: #00e5ff; font-family: 'Orbitron', sans-serif; font-size: 10px; }

  input.cell {
    width: 42px; background: #000; color: #00ff87; border: 1px solid #222;
    text-align: center; padding: 5px 1px; font-family: 'Rajdhani', sans-serif;
    font-weight: 700; font-size: 14px; border-radius: 4px;
  }
  input.cell:focus { outline: none; border-color: #ff0055; color: #fff; box-shadow: 0 0 8px #ff0055; }
  input.dwell-cell { color: #00e5ff; width: 75px; }
  input.dwell-cell:focus { border-color: #00e5ff; color: #fff; box-shadow: 0 0 8px #00e5ff; }

  .btn-save {
    background: linear-gradient(135deg, #ff0055, #ff5500); color: #fff;
    font-family: 'Orbitron', sans-serif; font-weight: 900; border: none;
    padding: 14px 20px; font-size: 14px; border-radius: 8px; cursor: pointer;
    width: 100%; letter-spacing: 2px; box-shadow: 0 0 20px rgba(255, 0, 85, 0.5); transition: 0.2s;
  }
  .btn-save:hover { transform: scale(1.01); box-shadow: 0 0 30px rgba(255, 0, 85, 0.8); }
</style>
</head><body>

<div class="marquee-box">
  <div class="marquee-inner">
    <div class="marquee-text">🏁 TEAM PATAS KUDUS SATRIA CLUB — TCI IGNITION MAP — CUSTOM TUNING SYSTEM — 🏁</div>
  </div>
</div>

<div class="header"><h1>TEAM PATAS KUDUS</h1></div>

<div class="container">
  <div class="timeout-warning">WIFI AUTO-SLEEP AKTIF DALAM 3 MENIT</div>
  
  <div class="card">
    <div class="controls-grid">
      <div>
        <label>SLOT MAP CUSTOM</label>
        <select id="slotSelect" onchange="loadMap()">
          <option value="0">MAP CUSTOM 1</option>
          <option value="1">MAP CUSTOM 2</option>
          <option value="2">MAP CUSTOM 3</option>
          <option value="3">MAP CUSTOM 4</option>
          <option value="4">MAP CUSTOM 5</option>
        </select>
      </div>
      <div>
        <label>LIMIT RPM (HARD CUT)</label>
        <input type="number" id="rpmLimit" class="cfg" value="12500" step="100">
      </div>
    </div>
  </div>

  <div class="tab-buttons">
    <button class="tab-btn active" id="btnTabDegree" onclick="switchTab('degree')">MAP DERAJAT (3D)</button>
    <button class="tab-btn" id="btnTabDwell" onclick="switchTab('dwell')">DWELL PER STEP (RPM)</button>
  </div>

  <div class="chart-card">
    <div class="chart-title" id="chartTitle">GRAFIK KURVA PENGAPIAN (DEGREE VS RPM)</div>
    <canvas id="tuningChart" style="width:100%; height:180px;"></canvas>
  </div>

  <div id="tabDegree" class="table-responsive">
    <table>
      <thead><tr><th>RPM/TPS</th><th>0%</th><th>25%</th><th>50%</th><th>75%</th><th>100%</th></tr></thead>
      <tbody id="mapBody"></tbody>
    </table>
  </div>

  <div id="tabDwell" class="table-responsive" style="display:none;">
    <table>
      <thead><tr><th>STEP RPM</th><th>DWELL TIME (&mu;s)</th><th>KETERANGAN</th></tr></thead>
      <tbody id="dwellBody"></tbody>
    </table>
  </div>

  <button class="btn-save" onclick="saveAndSend()">SIMPAN & FLASH KE ECU</button>
</div>

<script>
const rpmLabels = ["1k","2k","3k","4k","5k","6k","7k","8k","9k","10k","11k","12k","13k","14k","15k"];

function switchTab(tab) {
  if (tab === 'degree') {
    document.getElementById('tabDegree').style.display = 'block';
    document.getElementById('tabDwell').style.display = 'none';
    document.getElementById('btnTabDegree').classList.add('active');
    document.getElementById('btnTabDwell').classList.remove('active');
    document.getElementById('chartTitle').innerText = "GRAFIK KURVA PENGAPIAN (DEGREE VS RPM)";
  } else {
    document.getElementById('tabDegree').style.display = 'none';
    document.getElementById('tabDwell').style.display = 'block';
    document.getElementById('btnTabDegree').classList.remove('active');
    document.getElementById('btnTabDwell').classList.add('active');
    document.getElementById('chartTitle').innerText = "GRAFIK KURVA DWELL TIME (DWELL VS RPM)";
  }
  drawChart();
}

function drawChart() {
  const canvas = document.getElementById('tuningChart');
  const ctx = canvas.getContext('2d');
  
  canvas.width = canvas.clientWidth;
  canvas.height = canvas.clientHeight;
  
  const w = canvas.width; const h = canvas.height;
  const padL = 30, padR = 15, padT = 15, padB = 25;
  const plotW = w - padL - padR; const plotH = h - padT - padB;
  
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = '#1a1a24'; ctx.lineWidth = 1; ctx.fillStyle = '#666'; ctx.font = '9px Orbitron, sans-serif';
  
  let currentTabIsDegree = document.getElementById('tabDegree').style.display !== 'none';
  
  if (currentTabIsDegree) {
    let minY = 0, maxY = 450; 
    for(let v = 0; v <= 400; v += 100) {
      let y = padT + plotH - ((v - minY) / (maxY - minY)) * plotH;
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
      ctx.fillText((v/10) + '°', 5, y + 3);
    }
    for(let r = 0; r < 15; r += 2) {
      let x = padL + (r / 14) * plotW;
      ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + plotH); ctx.stroke();
      ctx.fillText(rpmLabels[r], x - 8, h - 5);
    }
    const colors = ['#00e5ff', '#00ff87', '#ffea00', '#ff9100', '#ff0055'];
    for(let t = 0; t < 5; t++) {
      ctx.strokeStyle = colors[t]; ctx.lineWidth = 2; ctx.beginPath();
      for(let r = 0; r < 15; r++) {
        let el = document.getElementById(`c_${r}_${t}`);
        let val = el ? (parseInt(el.value) || 0) : 100;
        let x = padL + (r / 14) * plotW;
        let y = padT + plotH - ((val - minY) / (maxY - minY)) * plotH;
        if(r === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
  } else {
    let minY = 1000, maxY = 5000;
    for(let v = 1000; v <= 5000; v += 1000) {
      let y = padT + plotH - ((v - minY) / (maxY - minY)) * plotH;
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
      ctx.fillText((v/1000) + 'k', 5, y + 3);
    }
    for(let r = 0; r < 15; r += 2) {
      let x = padL + (r / 14) * plotW;
      ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + plotH); ctx.stroke();
      ctx.fillText(rpmLabels[r], x - 8, h - 5);
    }
    ctx.strokeStyle = '#00e5ff'; ctx.lineWidth = 2.5; ctx.beginPath();
    for(let r = 0; r < 15; r++) {
      let el = document.getElementById(`d_${r}`);
      let val = el ? (parseInt(el.value) || 3200) : 3200;
      let x = padL + (r / 14) * plotW;
      let y = padT + plotH - ((val - minY) / (maxY - minY)) * plotH;
      if(r === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
}

function loadMap() {
  let slot = document.getElementById('slotSelect').value;
  fetch('/getMap?slot=' + slot).then(r => r.json()).then(data => {
    document.getElementById('rpmLimit').value = data.rpmLimit;
    
    let tbodyMap = document.getElementById('mapBody'); tbodyMap.innerHTML = '';
    for(let r = 0; r < 15; r++) {
      let tr = document.createElement('tr');
      let tdRpm = document.createElement('td');
      tdRpm.style.color = '#00e5ff'; tdRpm.innerText = rpmLabels[r]; tr.appendChild(tdRpm);
      for(let t = 0; t < 5; t++) {
        let td = document.createElement('td');
        let val = (data.map && data.map[r]) ? data.map[r][t] : 100;
        td.innerHTML = `<input type="number" class="cell" id="c_${r}_${t}" value="${val}" oninput="drawChart()">`;
        tr.appendChild(td);
      }
      tbodyMap.appendChild(tr);
    }

    let tbodyDwell = document.getElementById('dwellBody'); tbodyDwell.innerHTML = '';
    for(let r = 0; r < 15; r++) {
      let tr = document.createElement('tr');
      let tdRpm = document.createElement('td'); tdRpm.style.color = '#00e5ff'; tdRpm.innerText = rpmLabels[r] + ' RPM';
      let tdDwell = document.createElement('td');
      let dVal = (data.dwell && data.dwell[r]) ? data.dwell[r] : 3200;
      tdDwell.innerHTML = `<input type="number" class="cell dwell-cell" id="d_${r}" value="${dVal}" step="50" oninput="drawChart()">`;
      let tdDesc = document.createElement('td'); tdDesc.style.color = '#888'; tdDesc.style.fontSize = '11px';
      tdDesc.innerText = r < 4 ? 'Low RPM' : (r < 10 ? 'Mid RPM' : 'High RPM');
      tr.appendChild(tdRpm); tr.appendChild(tdDwell); tr.appendChild(tdDesc); tbodyDwell.appendChild(tr);
    }
    drawChart();
  });
}

function saveAndSend() {
  let slot = parseInt(document.getElementById('slotSelect').value);
  let payload = {
    slot: slot, rpmLimit: parseInt(document.getElementById('rpmLimit').value) || 12500, map: [], dwell: []
  };

  for(let r = 0; r < 15; r++) {
    let row = [];
    for(let t = 0; t < 5; t++) { row.push(parseInt(document.getElementById(`c_${r}_${t}`).value) || 100); }
    payload.map.push(row);
  }
  for(let r = 0; r < 15; r++) { payload.dwell.push(parseInt(document.getElementById(`d_${r}`).value) || 3200); }
  
  fetch('/saveTuning', {
    method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(payload)
  }).then(r => r.text()).then(msg => alert('Map Custom ' + (slot+1) + ' Berhasil Disimpan & Dikirim ke ECU!'));
}

window.onload = loadMap; window.onresize = drawChart;
</script>
</body></html>
)rawliteral";

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ 
    lastActivityTime = millis(); // Reset Timer saat user membuka web
    request->send_P(200, "text/html", htmlUI); 
  });
  
  server.on("/getMap", HTTP_GET, [](AsyncWebServerRequest *request){
    lastActivityTime = millis(); // Reset Timer saat data map dimuat
    uint8_t slot = request->hasParam("slot") ? request->getParam("slot")->value().toInt() : 0;
    if (slot > 4) slot = 0;
    
    DynamicJsonDocument doc(4096);
    doc["rpmLimit"] = customMapSlots[slot].rpmLimit;
    
    JsonArray arrMap = doc.createNestedArray("map");
    for (int r = 0; r < NUM_RPM_POINTS; r++) {
      JsonArray row = arrMap.createNestedArray();
      for (int t = 0; t < NUM_TPS_POINTS; t++) row.add(customMapSlots[slot].mapData[r][t]);
    }
    JsonArray arrDwell = doc.createNestedArray("dwell");
    for (int r = 0; r < NUM_RPM_POINTS; r++) arrDwell.add(customMapSlots[slot].dwellData[r]);

    String response; serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  AsyncCallbackJsonWebHandler* handler = new AsyncCallbackJsonWebHandler("/saveTuning", [](AsyncWebServerRequest *request, JsonVariant &json) {
    lastActivityTime = millis(); // Reset Timer saat save map
    JsonObject obj = json.as<JsonObject>();
    uint8_t slot = obj["slot"].as<uint8_t>();
    if (slot > 4) slot = 0;
    
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
    sendCommandToS3(0x02, slot); 
    request->send(200, "text/plain", "Saved & Sent");
  });
  
  server.addHandler(handler);
  server.begin();
}

void setup() {
  // EFISIENSI 1: Turunkan Clock CPU dari 160MHz ke 80MHz (Suhu akan turun drastis)
  setCpuFrequencyMhz(80); 
  
  Serial1.begin(250000, SERIAL_8N1, RX_PIN, TX_PIN);
  
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("ECU_SATRIA_FU", "kudus1234");
  
  // EFISIENSI 2: Tetap pertahankan TX Power di tingkat medium (50)
  esp_wifi_set_max_tx_power(50); 
  
  esp_wifi_set_mac(WIFI_IF_STA, transmitterMac); 
  if (esp_now_init() == ESP_OK) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, displayMacAddress, 6);
    esp_now_add_peer(&peerInfo);
    esp_now_register_recv_cb(OnEspNowRecv); 
  }

  loadMapFromNVS();
  setupWebServer();
  
  lastActivityTime = millis(); // Mulai perhitungan timer saat boot
  delay(1000); sendCommandToS3(0x02, 0); 
}

void loop() {
  parseUART(); 
  
  // EFISIENSI 3: Auto-Kill WiFi & Web Server setelah 3 Menit idle
  if (isWifiActive && (millis() - lastActivityTime > WIFI_TIMEOUT_MS)) {
    server.end();                  // Matikan Web Server
    WiFi.softAPdisconnect(true);   // Matikan jaringan WiFi Access Point
    isWifiActive = false;          // Kunci state agar tidak dieksekusi berulang
    // Mode STA tetap hidup di latar belakang untuk komunikasi ESP-NOW ke display
  }
  
  // Memberi waktu "napas" pada OS agar suhu tetap dingin
  delay(1); 
}
