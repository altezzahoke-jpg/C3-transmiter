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

// --- DATA BAWAAN (MAP DAILY S3) ---
const int16_t mapDaily3DBase[NUM_RPM_POINTS][NUM_TPS_POINTS] = {
  {100, 100, 100, 100, 100}, {100, 100, 100, 100, 100}, {140, 150, 160, 170, 180},
  {180, 200, 220, 230, 240}, {220, 240, 260, 270, 280}, {250, 270, 290, 300, 310},
  {280, 300, 320, 330, 330}, {300, 320, 340, 350, 350}, {310, 330, 350, 350, 350},
  {310, 330, 350, 350, 350}, {300, 320, 340, 340, 340}, {280, 300, 320, 320, 320},
  {260, 280, 300, 300, 300}, {250, 250, 280, 280, 280}, {250, 250, 250, 250, 250}
};

// --- STRUKTUR DATA TUNING ---
struct CustomTuning {
  int16_t mapData[NUM_RPM_POINTS][NUM_TPS_POINTS];
  uint16_t rpmLimit;
  uint16_t dwellUs;
};

CustomTuning customMapSlots[5]; // 5 Slot Map
uint8_t activeUIMapSlot = 0;

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

// UPDATE: Ditambahkan rpmLimit dan dwellUs
struct __attribute__((packed)) CommandPacketToS3 {
  uint16_t header;       
  uint8_t  cmdType;      
  uint8_t  slotOrMode;   
  int16_t  mapData[NUM_RPM_POINTS][NUM_TPS_POINTS]; 
  uint16_t rpmLimit;     
  uint16_t dwellUs;
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
    cmd.rpmLimit = customMapSlots[slot].rpmLimit;
    cmd.dwellUs = customMapSlots[slot].dwellUs;
  } else {
    memset(cmd.mapData, 0, sizeof(cmd.mapData)); 
    cmd.rpmLimit = 12500;
    cmd.dwellUs = 3200;
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
      customMapSlots[s].dwellUs = 3200;
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

// --- TAMPILAN WEB UI ---
const char* htmlUI = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ECU Tuner Pro</title>
<style>
  body{font-family:Arial,sans-serif;background:#121212;color:#fff;text-align:center;margin:10px;padding:0;}
  h2{color:#00e676;margin-bottom:10px;}
  .container{max-width:600px;margin:0 auto;}
  .controls{background:#1e1e1e;padding:15px;border-radius:8px;margin-bottom:15px;display:flex;flex-wrap:wrap;justify-content:space-between;align-items:center;border:1px solid #333;}
  select, input[type=number].cfg{background:#000;color:#00e676;border:1px solid #444;padding:8px;font-size:14px;border-radius:4px;}
  label{font-size:14px;font-weight:bold;margin-right:10px;}
  .cfg-group{margin:5px 0;}
  table{width:100%;border-collapse:collapse;margin-bottom:15px;background:#1e1e1e;}
  th,td{border:1px solid #333;padding:4px;text-align:center;font-size:12px;}
  th{background:#292929;color:#00e676;}
  input.cell{width:42px;background:#000;color:#00e676;border:1px solid #444;text-align:center;padding:5px;font-size:13px;border-radius:4px;}
  input.cell:focus{outline:none;border-color:#00e676;}
  button{background:#00e676;color:#000;font-weight:bold;border:none;padding:12px 20px;font-size:14px;border-radius:6px;cursor:pointer;margin:5px;width:100%;max-width:250px;}
  button:hover{opacity:0.9;}
</style>
</head><body>
<h2>ECU TUNER PRO</h2>
<div class="container">
  <div class="controls">
    <div class="cfg-group">
      <label>SLOT MAP:</label>
      <select id="slotSelect" onchange="loadMap()">
        <option value="0">Custom Map 1</option><option value="1">Custom Map 2</option>
        <option value="2">Custom Map 3</option><option value="3">Custom Map 4</option>
        <option value="4">Custom Map 5</option>
      </select>
    </div>
    <div class="cfg-group"><label>RPM LIMIT:</label><input type="number" id="rpmLimit" class="cfg" value="12500"></div>
    <div class="cfg-group"><label>DWELL (us):</label><input type="number" id="dwellUs" class="cfg" value="3200"></div>
  </div>

  <div style="overflow-x:auto;">
    <table>
      <thead><tr><th>RPM/TPS</th><th>0%</th><th>25%</th><th>50%</th><th>75%</th><th>100%</th></tr></thead>
      <tbody id="mapBody"></tbody>
    </table>
  </div>
  
  <button onclick="saveAndSend()">SIMPAN & KIRIM KE ECU</button>
</div>

<script>
const rpmLabels = ["1000","2000","3000","4000","5000","6000","7000","8000","9000","10000","11000","12000","13000","14000","15000"];

function loadMap() {
  let slot = document.getElementById('slotSelect').value;
  fetch('/getMap?slot=' + slot).then(r => r.json()).then(data => {
    document.getElementById('rpmLimit').value = data.rpmLimit;
    document.getElementById('dwellUs').value = data.dwellUs;
    let tbody = document.getElementById('mapBody');
    tbody.innerHTML = '';
    for(let r = 0; r < 15; r++) {
      let tr = document.createElement('tr');
      let tdRpm = document.createElement('td');
      tdRpm.innerText = rpmLabels[r];
      tr.appendChild(tdRpm);
      for(let t = 0; t < 5; t++) {
        let td = document.createElement('td');
        let val = (data.map && data.map[r]) ? data.map[r][t] : 100;
        td.innerHTML = `<input type="number" class="cell" id="c_${r}_${t}" value="${val}">`;
        tr.appendChild(td);
      }
      tbody.appendChild(tr);
    }
  });
}

function saveAndSend() {
  let slot = parseInt(document.getElementById('slotSelect').value);
  let payload = {
    slot: slot,
    rpmLimit: parseInt(document.getElementById('rpmLimit').value) || 12500,
    dwell: parseInt(document.getElementById('dwellUs').value) || 3200,
    map: []
  };
  for(let r = 0; r < 15; r++) {
    let row = [];
    for(let t = 0; t < 5; t++) {
      row.push(parseInt(document.getElementById(`c_${r}_${t}`).value) || 100);
    }
    payload.map.push(row);
  }
  
  fetch('/saveTuning', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(payload)
  }).then(r => r.text()).then(msg => alert('Data Map ' + (slot+1) + ' Berhasil Disimpan & Dikirim!'));
}
window.onload = loadMap;
</script>
</body></html>
)rawliteral";

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ request->send_P(200, "text/html", htmlUI); });
  
  server.on("/getMap", HTTP_GET, [](AsyncWebServerRequest *request){
    uint8_t slot = 0;
    if (request->hasParam("slot")) { slot = request->getParam("slot")->value().toInt(); }
    if (slot > 4) slot = 0;
    
    DynamicJsonDocument doc(4096);
    doc["rpmLimit"] = customMapSlots[slot].rpmLimit;
    doc["dwellUs"] = customMapSlots[slot].dwellUs;
    JsonArray arr = doc.createNestedArray("map");
    for (int r = 0; r < NUM_RPM_POINTS; r++) {
      JsonArray row = arr.createNestedArray();
      for (int t = 0; t < NUM_TPS_POINTS; t++) { row.add(customMapSlots[slot].mapData[r][t]); }
    }
    String response; serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  AsyncCallbackJsonWebHandler* handler = new AsyncCallbackJsonWebHandler("/saveTuning", [](AsyncWebServerRequest *request, JsonVariant &json) {
    JsonObject obj = json.as<JsonObject>();
    uint8_t slot = obj["slot"].as<uint8_t>();
    if (slot > 4) slot = 0;
    
    customMapSlots[slot].rpmLimit = obj["rpmLimit"].as<uint16_t>();
    customMapSlots[slot].dwellUs = obj["dwell"].as<uint16_t>();
    
    JsonArray arr = obj["map"].as<JsonArray>(); int r = 0;
    for(JsonVariant row : arr) {
      if(r >= NUM_RPM_POINTS) break; int t = 0;
      for(JsonVariant val : row.as<JsonArray>()) { 
        if(t >= NUM_TPS_POINTS) break; 
        customMapSlots[slot].mapData[r][t] = val.as<int16_t>(); t++; 
      } r++;
    }
    
    saveMapToNVS(); 
    sendCommandToS3(0x02, slot); 
    request->send(200, "text/plain", "Saved & Sent");
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
