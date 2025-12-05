# Building a Wireless Sensor Network Using ESP32, ESP-NOW, and MQTT

We built an energy-aware wireless sensor network with ESP32 nodes using ESP-NOW to reach a gateway that forwards data to MQTT for real-time dashboards in Node-RED. Each node measures temperature and humidity with a DHT11, shows local status on LEDs, and sends JSON messages the gateway relays to the broker. This README explains the flow, setup, and validation steps so you can reproduce the system.

The design fits smart agriculture, indoor tracking, and other distributed monitoring that needs low power and low latency. ESP-NOW keeps node-to-gateway messaging fast and router-free; MQTT makes the data easy to consume in dashboards or the cloud.

---

## What You Get

- A repeatable recipe for three sensor nodes and one gateway using ESP-NOW + MQTT.
- Step-by-step wiring, firmware config, and flashing instructions.
- A clear path: nodes → ESP-NOW → gateway → MQTT → Node-RED.
- Power math to estimate battery life.
- Troubleshooting and security notes drawn from the report.

---

## Data Flow

1) Every ~5 seconds a node reads the DHT11, sets its NeoPixel color, and decides if there’s an alert.  
2) It sends a tiny JSON over ESP-NOW to the gateway.  
3) The gateway stamps it, tracks the hottest node, and republishes to MQTT.  
4) Node-RED (or any MQTT client) listens on `home/sensor_data` to drive gauges, charts, and alerts.  
5) If a node goes quiet, the gateway marks it offline and can publish that status.

---

## Project at a Glance

- 3x ESP32 sensor nodes: DHT11 temp/humidity, NeoPixel for temperature color, red LED for alerts, ESP-NOW transport
- 1x ESP32 gateway: collects ESP-NOW packets, republishes to MQTT, tracks node health, highlights hottest node
- MQTT broker (Mosquitto) for pub/sub
- Node-RED dashboard for gauges, charts, node status, highest-temperature alert, and global alert indicator
- JSON payloads so the data can be reused by other services

---

## System Overview

- Sensor Nodes  
  - DHT11 temperature and humidity sampling every 5 s (tunable).  
  - NeoPixel shows local temperature band: blue (cool), green (normal), red (hot).  
  - Red LED blinks when any node reports `alert=1`, so urgency spreads across the mesh.  
  - ESP-NOW handles node-to-gateway links with minimal latency and no router.  
  - JSON payload keeps the format simple for anything that consumes it.
- Gateway Node  
  - Receives all ESP-NOW packets and validates payloads.  
  - Publishes JSON to MQTT (`home/sensor_data`).  
  - Maintains per-node heartbeat to mark ONLINE/OFFLINE status.  
  - Tracks the highest-temperature node for dashboard highlighting.  
  - Serial output helps with bring-up and field debugging.
- MQTT + Node-RED  
  - Mosquitto broker transports messages to any subscriber.  
  - Node-RED dashboard renders gauges, charts, online/offline state, and alert banners.

---

## Why ESP-NOW?

- Extremely low latency (2-3 ms typical)  
- Works without Wi-Fi association or DHCP  
- Low energy footprint compared with full Wi-Fi stack  
- Peer-to-peer broadcast and unicast support  
- Keeps sensor mesh independent of internet; gateway uses Wi-Fi only for MQTT

---

## Architecture Diagram

![System architecture showing ESP32 nodes sending ESP-NOW data to a gateway, which publishes JSON to MQTT and feeds a Node-RED dashboard](./IOT%20PICT.png)

---

## Working Video

[![Watch the working video](https://img.youtube.com/vi/FFtykHSSWy4/hqdefault.jpg)](https://www.youtube.com/watch?v=FFtykHSSWy4)

---

## Bill of Materials

| Item                     | Qty | Notes                                    |
| ------------------------ | --- | ---------------------------------------- |
| ESP32 DevKit (any WROOM) | 4   | 3 sensors + 1 gateway                    |
| DHT11 temperature/humidity sensor | 3 | One per sensor node                    |
| NeoPixel (single or small ring)   | 3 | GPIO 4 recommended                     |
| Red LED + 220 ohm resistor | 3 | Alert indicator, GPIO 6 recommended      |
| USB cables / 5V supply   | 4   | Or Li-Ion with regulator                 |
| Breadboard + jumpers     | 3   | For quick wiring                         |
| Mosquitto MQTT broker    | 1   | Local or cloud                           |
| Node-RED                 | 1   | Dashboard runtime                        |

---

## Wiring (per Sensor Node)

| Connection          | ESP32 Pin (example) |
| ------------------- | ------------------- |
| DHT11 VCC           | 3V3                 |
| DHT11 GND           | GND                 |
| DHT11 DATA          | GPIO 16             |
| NeoPixel DIN        | GPIO 4              |
| NeoPixel VCC/GND    | 5V (or 3V3) / GND   |
| Red LED anode (+R)  | GPIO 6 via 220 ohm  |
| Red LED cathode     | GND                 |

Gateway wiring is the same if you keep a local DHT11 for reference; otherwise leave it off.

---

## Firmware Layout and Config Hints

- Define unique ESP-NOW peer MAC addresses for each sensor node on the gateway.
- Shared payload schema (per packet):  
  `{"node": <int>, "temp": <float C>, "hum": <float %>, "alert": <0|1>}`
- Suggested constants:  
  - `WIFI_SSID`, `WIFI_PASSWORD` (gateway only, for MQTT uplink)  
  - `MQTT_BROKER`, `MQTT_PORT` (1883), `MQTT_TOPIC="home/sensor_data"`  
  - `ESP_NOW_CHANNEL` (e.g., 1) kept consistent across devices  
  - `SAMPLE_INTERVAL_MS` (e.g., 5000)
- Libraries (Arduino): `WiFi.h`, `esp_now.h`, `PubSubClient`, `DHT`, `Adafruit_NeoPixel`, `ArduinoJson`.

---

## Build and Flash

1) Install ESP32 boards package in Arduino IDE (or use PlatformIO with `platform = espressif32`).  
2) Install required libraries listed above.  
3) Set the board to your ESP32 DevKit and correct COM port.  
4) Flash the sensor-node firmware to three boards, changing the node ID and peer MAC list for each.  
5) Flash the gateway firmware, adding the MAC addresses of all nodes and your Wi-Fi + MQTT settings.  
6) Open the Serial Monitor (115200) on the gateway to confirm ESP-NOW pairing and MQTT publishes.

---

## End-to-End Setup (Condensed)

1) Wire one sensor node fully and flash the sensor sketch with `nodeId=1`. Confirm serial output and ESP-NOW sends.  
2) Wire and flash nodes 2 and 3, updating `nodeId` and peer lists each time.  
3) Flash the gateway with all peer MACs and your Wi-Fi + MQTT credentials; watch for successful pairing and MQTT publishes.  
4) Start Mosquitto (or point to your broker) and subscribe with `mosquitto_sub -t home/sensor_data -v` to verify payloads.  
5) Import/build the Node-RED flow, bind gauges/charts to `home/sensor_data`, and deploy.  
6) Heat one sensor gently to trigger `alert=1`; watch LEDs, MQTT payload, and the dashboard alert update together.

---

## MQTT Topics and Payloads

- Topic: `home/sensor_data`
- Payload example:

```json
{"node":1,"temp":31.20,"hum":58.00,"alert":1}
```

- Gateway may emit status/heartbeat (optional):  
  - `home/sensor_status`: `{"node":1,"online":true,"rssi":-61}`  
  - `home/highest_temp`: `{"node":3,"temp":33.1}`

---

## Node Behavior

- Sample DHT11 every 5 seconds (adjust `SAMPLE_INTERVAL_MS` for battery vs. responsiveness).  
- Color set on NeoPixel: blue < 22 C, green 22-30 C, red > 30 C (tune thresholds).  
- If `temp` exceeds threshold, set `alert=1` and blink the red LED locally; other nodes receiving `alert=1` also blink their red LED.  
- Retransmit on ESP-NOW if acknowledgment fails (gateway should ack or use acks-on).  
- Sleep between sends to hit low duty cycle.

---

## Gateway Behavior

- Listens on ESP-NOW, validates packets, and records last-heard timestamp per node.  
- Publishes every valid packet to MQTT immediately.  
- Every ~5 seconds, marks nodes ONLINE/OFFLINE based on timeout and publishes status (optional).  
- Tracks hottest node and raises a dashboard alert when a threshold is crossed.  
- Can rebroadcast alerts or blink a local LED for a quick at-a-glance check during field visits.

---

## Node-RED Dashboard Quick Start

1) Install Node-RED and the dashboard nodes (`node-red-dashboard`).  
2) Create an MQTT-in node subscribed to `home/sensor_data`.  
3) Split/route by `msg.payload.node` to individual gauges and charts (temp and humidity per node).  
4) Derive highest-temp banner and global alert indicator when any `alert=1`.  
5) Optional: add MQTT-in for `home/sensor_status` to drive online/offline indicators.  
6) Deploy and watch live updates in the browser dashboard.

---

## Field Calibration and Thresholds

- Tune temperature bands for your climate (e.g., blue < 20 C, green 20-28 C, red > 28 C for indoor comfort; adjust higher for greenhouses).  
- Validate DHT11 readings against a reference thermometer for a few minutes; apply a small offset in code if needed.  
- If using batteries, dim the NeoPixel and reduce alert blink duration to save power.  
- Set the offline timeout to 2-3x the send interval to avoid false offline flags.

---

## Operations and Maintenance

- Log MQTT to disk (e.g., Telegraf/InfluxDB) if you need history beyond the dashboard.  
- Schedule a weekly reboot for lab demos; for field deployments, enable watchdog timers on all boards.  
- Keep a record of node MAC addresses, physical placement, and GPIO mappings for replacements.  
- For battery nodes, periodically measure pack voltage (ADC) and publish to MQTT for early low-battery alerts.

---

## Network Configuration Cheatsheet

- Keep all ESP-NOW devices on the same Wi-Fi channel (e.g., channel 1).  
- Fix the gateway’s Wi-Fi channel before pairing to avoid channel hopping.  
- If your Wi-Fi AP forces 40 MHz, lock it to 20 MHz for more reliable ESP-NOW.  
- Use static IP on the gateway for simpler MQTT broker allowlists.  
- If using WPA2 Enterprise, keep the gateway on a separate SSID dedicated to IoT.

---

## Example Firmware (real sketches)

Sensor node (New_Slave_2.ino):
```cpp
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Adafruit_NeoPixel.h>
#include <DHT.h>

#define NODE_ID         2

#define WIFI_CHANNEL    11
#define DHTPIN          5
#define DHTTYPE         DHT11

#define NEOPIX_PIN      4
#define NEOPIX_COUNT    1
#define GLOBAL_ALERT_LED 6

#define TEMP_LOW   20.0
#define TEMP_HIGH  30.0

// Gateway MAC
uint8_t GATEWAY_MAC[] = {0x7C, 0xDF, 0xA1, 0xBE, 0xAD, 0xB4};

typedef struct {
  uint8_t nodeId;
  float   temp;
  float   hum;
} SensorPacket;

DHT dht(DHTPIN, DHTTYPE);
Adafruit_NeoPixel pixel(NEOPIX_COUNT, NEOPIX_PIN, NEO_GRB + NEO_KHZ800);

void onEspNowRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  String msg;
  for (int i = 0; i < len; i++) {
    msg += (char)data[i];
  }
  Serial.print("[Node ");
  Serial.print(NODE_ID);
  Serial.print("] ESP-NOW message: ");
  Serial.println(msg);

  if (msg == "GLOBAL_ALERT_HIGH") {
    digitalWrite(GLOBAL_ALERT_LED, HIGH);
    Serial.println("Global alert: HIGH TEMP - Red LED ON");
  } else if (msg == "GLOBAL_ALERT_CLEAR") {
    digitalWrite(GLOBAL_ALERT_LED, LOW);
    Serial.println("Global alert cleared - Red LED OFF");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(GLOBAL_ALERT_LED, OUTPUT);
  digitalWrite(GLOBAL_ALERT_LED, LOW);

  dht.begin();

  pixel.begin();
  pixel.setBrightness(150);
  pixel.clear();
  pixel.show();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    return;
  }
  Serial.println("ESP-NOW init OK");

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, GATEWAY_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add gateway peer");
  }

  esp_now_register_recv_cb(onEspNowRecv);
}

void updateNeoPixel(float temp) {
  if (temp < TEMP_LOW) {
    pixel.setPixelColor(0, pixel.Color(0, 0, 255)); // cool
  } else if (temp > TEMP_HIGH) {
    pixel.setPixelColor(0, pixel.Color(255, 0, 0)); // hot
  } else {
    pixel.setPixelColor(0, pixel.Color(0, 255, 0)); // normal
  }
  pixel.show();
}

void loop() {
  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();

  if (isnan(temp) || isnan(hum)) {
    Serial.println("DHT read error");
    delay(2000);
    return;
  }

  updateNeoPixel(temp);

  SensorPacket pkt;
  pkt.nodeId = NODE_ID;
  pkt.temp   = temp;
  pkt.hum    = hum;

  esp_err_t res = esp_now_send(GATEWAY_MAC, (uint8_t*)&pkt, sizeof(pkt));

  Serial.print("[Node ");
  Serial.print(NODE_ID);
  Serial.print("] Sent - T=");
  Serial.print(temp);
  Serial.print("C, H=");
  Serial.print(hum);
  Serial.print("%, send=");
  Serial.println(res == ESP_OK ? "OK" : "ERR");

  delay(5000); // 5 seconds between readings
  Serial.println("Entering sleep...");
  esp_sleep_enable_timer_wakeup(10 * 1000000);
  esp_light_sleep_start();

  Serial.println("Woke up from sleep...");
}
```

Gateway (gateway_code_1.ino):
```cpp
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <PubSubClient.h>

#define WIFI_SSID     "Amitabh Mama"
#define WIFI_PASSWORD "gearboys@123"
#define WIFI_CHANNEL  11

#define MQTT_SERVER   "test.mosquitto.org"
#define MQTT_PORT     1883
#define MQTT_TOPIC    "home/sensor_data"

uint8_t NODE1_MAC[] = {0x84, 0xF7, 0x03, 0x12, 0xD0, 0x50};
uint8_t NODE2_MAC[] = {0x84, 0xF7, 0x03, 0x12, 0xA8, 0x4C};
uint8_t NODE3_MAC[] = {0x84, 0xF7, 0x03, 0x12, 0xC6, 0xFC};

typedef struct {
  uint8_t nodeId;
  float   temp;
  float   hum;
} SensorPacket;

WiFiClient espClient;
PubSubClient client(espClient);

float lastTemp[4] = {0,0,0,0};
float lastHum[4]  = {0,0,0,0};
bool  seen[4]     = {false,false,false,false};

bool globalAlert = false;
const float TEMP_HIGH = 30.0;

void sendGlobalAlert(bool high) {
  const char *msg = high ? "GLOBAL_ALERT_HIGH" : "GLOBAL_ALERT_CLEAR";
  int len = strlen(msg);

  Serial.print("Sending global alert: ");
  Serial.println(msg);

  esp_now_send(NODE1_MAC, (uint8_t*)msg, len);
  esp_now_send(NODE2_MAC, (uint8_t*)msg, len);
  esp_now_send(NODE3_MAC, (uint8_t*)msg, len);
}

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(SensorPacket)) {
    Serial.println("Unexpected ESP-NOW packet size");
    return;
  }

  SensorPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  if (pkt.nodeId < 1 || pkt.nodeId > 3) {
    Serial.println("Unknown nodeId");
    return;
  }

  int id = pkt.nodeId;
  lastTemp[id] = pkt.temp;
  lastHum[id]  = pkt.hum;
  seen[id]     = true;

  Serial.print("From node ");
  Serial.print(id);
  Serial.print(" - T=");
  Serial.print(pkt.temp);
  Serial.print("C, H=");
  Serial.print(pkt.hum);
  Serial.println("%");

  bool anyHigh = false;
  for (int i = 1; i <= 3; i++) {
    if (seen[i] && lastTemp[i] > TEMP_HIGH) {
      anyHigh = true;
      break;
    }
  }

  if (anyHigh != globalAlert) {
    globalAlert = anyHigh;
    sendGlobalAlert(globalAlert);
  }

  char json[200];
  snprintf(json, sizeof(json),
           "{\"node\":%d,\"temp\":%.2f,\"hum\":%.2f,\"alert\":%d}",
           id, pkt.temp, pkt.hum, (pkt.temp > TEMP_HIGH ? 1 : 0));

  if (client.connected()) {
    client.publish(MQTT_TOPIC, json);
  }

  Serial.print("MQTT JSON: ");
  Serial.println(json);
}

void setupWifi() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);

  Serial.print("Connecting to WiFi ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Gateway STA MAC: ");
  Serial.println(WiFi.macAddress());
}

void reconnectMqtt() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32C3_Gateway")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 5 seconds");
      delay(5000);
    }
  }
}

void addPeer(uint8_t *mac) {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
  }
}

void setupEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    while (true) delay(100);
  }
  Serial.println("ESP-NOW init OK");

  esp_now_register_recv_cb(onReceive);

  addPeer(NODE1_MAC);
  addPeer(NODE2_MAC);
  addPeer(NODE3_MAC);
}

void setup() {
  setupWifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  setupEspNow();
}

void loop() {
  if (!client.connected()) {
    reconnectMqtt();
  }
  client.loop();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 5000) {
    lastPrint = millis();
    Serial.println("---- Node summary ----");
    for (int i = 1; i <= 3; i++) {
      if (seen[i]) {
        Serial.print("Node ");
        Serial.print(i);
        Serial.print(": T=");
        Serial.print(lastTemp[i]);
        Serial.print(" H=");
        Serial.println(lastHum[i]);
      } else {
        Serial.print("Node ");
        Serial.print(i);
        Serial.println(": no data yet");
      }
    }
    Serial.print("Global alert: ");
    Serial.println(globalAlert ? "ON" : "OFF");
  }
}
```

---

## Testing and Validation Checklist

- Power-on each sensor node and confirm Serial shows DHT reads and ESP-NOW sends.  
- Verify NeoPixel color changes with gentle heat/cool (e.g., warm breath, cold pack).  
- Check gateway Serial: packets received, hottest node ID, MQTT publish success.  
- Subscribe with `mosquitto_sub -t home/sensor_data -v` and verify payload fields.  
- In Node-RED, confirm gauges update and alert banner flashes when threshold exceeded.  
- Unplug a node and confirm it is marked OFFLINE after timeout.  
- Battery test: measure sleep current; aim for sub-1 mA average.

---

## Deployment Modes

- **Lab demo**: all USB-powered, short intervals (5 s), verbose serial logging.  
- **Field trial**: battery-backed nodes, longer intervals (15–60 s), dim NeoPixel, disable debug prints.  
- **Production**: locked channel, static IP gateway, TLS MQTT if broker supports it, OTA updates enabled, watchdog timers on nodes and gateway.

---

## Security Notes

- Use a dedicated SSID/VLAN for IoT; restrict broker access to gateway IP only.  
- Prefer authenticated MQTT; if possible enable TLS or use a local broker on isolated LAN.  
- Avoid hardcoding credentials in plaintext when sharing code; keep them in `secrets.h`.  
- Rotate keys and regenerate ESP-NOW peer lists if devices are lost or stolen.  
- Disable unnecessary serial output before deployment to reduce info leakage.

---

## FAQ

- **Can I add more than 3 nodes?** Yes—ESP-NOW supports more peers; update peer lists and dashboard routing.  
- **Can I use DHT22?** Yes; adjust library calls and pinouts. Sampling can be faster and more accurate.  
- **What about OTA updates?** Add ArduinoOTA or ESP32HTTPUpdateServer on the gateway; nodes can use OTA if they occasionally join Wi-Fi.  
- **Do I need Internet?** No; ESP-NOW mesh and local MQTT work fully offline. Internet is only needed if you push data to cloud dashboards.  
- **Can I replace Node-RED?** Any MQTT-capable stack works (Home Assistant, Grafana/Telegraf/InfluxDB, custom apps).

---

## Duty Cycle and Energy Efficiency

Transmission every 5 seconds with ~17 ms active time.

```
Duty Cycle = (0.017 / 5.025) * 100 ~= 0.34%
Iavg = (180 mA * 0.0034) + (0.02 mA * 0.9966) ~= 0.632 mA
Battery Life ~= 1500 / 0.632 ~= 99 days (for a 1500 mAh 18650)
```

Tips: reduce sample rate, dim NeoPixel, and disable serial logging in production for even longer life.

---

## Troubleshooting

- ESP-NOW pairing fails: ensure all devices use the same Wi-Fi channel; confirm peer MACs are correct.  
- MQTT not publishing: verify broker IP/port, credentials, and Wi-Fi RSSI; ensure `client.loop()` runs frequently.  
- DHT11 read errors: add 10 k pull-up on data, increase interval to 2 s minimum, keep wiring short.  
- LED colors wrong: confirm NeoPixel power and data pin; verify GRB vs RGB order in code.  
- Offline status flapping: lengthen heartbeat timeout or lower send interval.

---

## Applications and Extensions

- Smart agriculture and greenhouses  
- Warehouse and cold-chain monitoring  
- Multi-room HVAC monitoring  
- Teaching labs for wireless sensor networks  
- Extend with: CO2/TVOC sensors, SD card logging, OTA updates, TLS MQTT, battery ADC for charge status.

---

## Conclusion

ESP32 boards with ESP-NOW and MQTT form a fast, energy-efficient, and scalable environmental monitoring network. With Node-RED visualization, cross-node alerts, and structured JSON, the system is ready for both academic demonstrations and real deployments, and can grow into automation or AI-driven analytics.
