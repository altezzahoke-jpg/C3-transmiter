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

// --- MAC ADDRESS TRANSMITTER (Disamakan dengan targetEcuAddress di C3 Layar) ---
uint8_t transmitterMac[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC}; 
uint8_t displayMacAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast ke C3 Layar

#define NUM_RPM_POINTS 15
#define NUM_TPS_POINTS 5
Preferences preferences;
AsyncWebServer server(80);

// --- STRUKTUR DATA (Identik dengan C3 Layar) ---
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
  uint16_t crc16;      
};

portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
TelemetryData currentTelemetry = {0xAA55, 0, 0, 100, 300, 126, 0, 0, 0, 0};
int16_t customMap[NUM_RPM_POINTS][NUM_TPS_POINTS];

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

void sendCommandToS3(uint8_t cmdType, uint8_t slotOrMode) {
  CommandPacketToS3 cmd;
  cmd.header = 0x55CC;
  cmd.cmdType = cmdType;
  cmd.slotOrMode = slotOrMode;
  if (cmdType == 0x02) memcpy(cmd.mapData, customMap, sizeof(customMap));
  else memset(cmd.mapData, 0, sizeof(cmd.mapData)); 
  
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
    if (cmd->header == 0xCC55) {
      uint16_t calcCrc = calculateCRC16(incomingData, sizeof(CommandData) - sizeof(uint16_t));
      if (cmd->crc16 == calcCrc) {
        sendCommandToS3(0x01, cmd->requestedMode); 
      }
    }
  }
}

void loadMapFromNVS() {
  preferences.begin("ecu_tuning", false);
  if (preferences.getBytesLength("customMap") == sizeof(customMap)) {
    preferences.getBytes("customMap", &customMap, sizeof(customMap));
  } else {
    for (int r = 0; r < NUM_RPM_POINTS; r++) for (int t = 0; t < NUM_TPS_POINTS; t++) customMap[r][t] = 100;
  }
  preferences.end();
}

void saveMapToNVS() {
  preferences.begin("ecu_tuning", false);
  preferences.putBytes("customMap", &customMap, sizeof(customMap));
  preferences.end();
}

const char* htmlUI = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{font-family:Arial;background:#111;color:#fff;text-align:center;} 
button{background:#18A3;color:#fff;border:none;padding:15px;font-size:18px;border-radius:8px;}</style>
</head><body>
<h2>ECU TUNER v3.0</h2><button onclick="sendSync()">KIRIM MAP KE ECU</button>
<script>function sendSync() { fetch('/syncMap', {method: 'POST'}).then(r => alert('Map Berhasil Dikirim!')); }</script></body></html>
)rawliteral";

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ request->send_P(200, "text/html", htmlUI); });
  server.on("/syncMap", HTTP_POST, [](AsyncWebServerRequest *request){
    sendCommandToS3(0x02, 0); request->send(200, "text/plain", "OK");
  });
  AsyncCallbackJsonWebHandler* handler = new AsyncCallbackJsonWebHandler("/saveTuning", [](AsyncWebServerRequest *request, JsonVariant &json) {
    JsonArray arr = json.as<JsonArray>(); int r = 0;
    for(JsonVariant row : arr) {
      if(r >= NUM_RPM_POINTS) break; int t = 0;
      for(JsonVariant val : row.as<JsonArray>()) { if(t >= NUM_TPS_POINTS) break; customMap[r][t] = val.as<int16_t>(); t++; } r++;
    }
    saveMapToNVS(); sendCommandToS3(0x02, 0); request->send(200, "text/plain", "Saved & Sent");
  });
  server.addHandler(handler);
  server.begin();
}

void setup() {
  Serial1.begin(250000, SERIAL_8N1, RX_PIN, TX_PIN);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("ECU_SATRIA_FU", "kudus1234");
  
  esp_wifi_set_mac(WIFI_IF_STA, transmitterMac); 
  
  if (esp_now_init() == ESP_OK) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, displayMacAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    esp_now_register_recv_cb(OnEspNowRecv); 
  }

  loadMapFromNVS();
  setupWebServer();
  delay(1000); sendCommandToS3(0x02, 0); 
}

void loop() {
  parseUART(); 
}
