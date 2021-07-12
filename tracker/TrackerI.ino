
#include <sstream>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <ArduinoJson.h>

#include "irk.h"

#ifndef WiFiSSID
#define WiFiSSID "YOUR_WIFI_SSID"
#endif

#ifndef WiFiPWD
#define WiFiPWD "YOUR_WIFI_PWD"
#endif

#ifndef MQTT_SERVER
#define MQTT_SERVER "YOUR_MQTT_SERVER_IP"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT 1883 //default
#endif

#ifndef MQTT_USER
#define MQTT_USER "YOUR_MQTT_SERVER_USER"
#endif

#ifndef MQTT_PWD
#define MQTT_PWD "YOUR_MQTT_SERVER_PWD"
#endif

#ifndef MQTT_TOPIC_ACTION
#define MQTT_TOPIC_ACTION "v1/trackeri/action" //default action
#endif

#ifndef MQTT_TOPIC_HEARBEAT
#define MQTT_TOPIC_HEARBEAT "v1/trackeri/heartbeat" //default heartbeat
#endif

#ifndef MQTT_TOPIC_STATE
#define MQTT_TOPIC_STATE "v1/trackeri/state" //default state
#endif

#ifndef BLE_SCAN_TIME
#define BLE_SCAN_TIME 5
#endif

#ifndef BLE_SCAN_DELAY
#define BLE_SCAN_DELAY 2000
#endif

//targetStateChangeCount
#ifndef USER_STATE_CHANGE_MAX_COUNT
#define USER_STATE_CHANGE_MAX_COUNT 3
#endif

#ifndef HEARTBEAT_DELAY
#define HEARTBEAT_DELAY 5
#endif

#ifndef BLESCANTASKTIMEOUT
#define BLESCANTASKTIMEOUT 60
#endif

#ifndef WIFI_CONNECT_TIMEOUT
#define WIFI_CONNECT_TIMEOUT 20 
#endif

#ifndef DEV_MODE
#define DEV_MODE 0
#endif

#if DEV_MODE
#define PRINT(str) Serial.print(str)
#define PRINTLN(str) Serial.println(str)
#define PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define PRINT(str)
#define PRINTLN(str)
#define PRINTF(str,...)
#endif


void networkTask( void *pvParameters );
void bleScanTask( void *pvParameters );
void(* resetFunc) (void) = 0; //reset function

uint8_t irk[][ESP_BT_OCTET16_LEN] = {
  //your irks
  //eg:
  //{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  //{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};

String users[] = {
  //users
  //eg:
  "Jack", 
  "LeeMing"
};

int irkListLength = sizeof(irk) / sizeof(irk[0]);



WiFiClient wifiClient;
PubSubClient pubSubClient(wifiClient);
BLEScan* trackerBLEScan;
char MAC_char_STA[18];

unsigned long networkTaskRunningTime;
unsigned long bleScanTaskRunningTime;

//signal led
int LED = 2;

// class TrackerAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
//     void onResult(BLEAdvertisedDevice advertisedDevice) {
//       PRINTF("Advertised Device: %s \n", advertisedDevice.toString().c_str());
//     }
// };

class UserDeviceInfo {
  public:
    uint8_t irk[ESP_BT_OCTET16_LEN];
    String userName;
    int currentState = -1;
    //int lostTargetCount = 0;
    int targetStateChangeCount = 0;
};
// UserDeviceInfo userDeviceInfos[irkListLength];
UserDeviceInfo *userDeviceInfos;

void setup() {
#if DEV_MODE
  Serial.begin(115200);
#endif
  initData();

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  //start network task
  newNetWorkTask();

  //start ble scan task
  newBLEScanTask();

  //init led
  pinMode(LED, OUTPUT);
  

}

/**
   newNetWorkTask
*/
void newNetWorkTask() {
  xTaskCreatePinnedToCore(
    networkTask,
    "networkTask",
    10000,
    NULL,
    1,
    NULL,
    0
  );
}

/**
   newBLEScanTask
*/
void newBLEScanTask() {
  xTaskCreatePinnedToCore(
    bleScanTask,
    "bleScanTask",
    10000,
    NULL,
    1,
    NULL,
    1
  );
}

void networkTask(void *pvParameters) {
  //setup
  initWiFi();
  initMQTT();

  //loop
  for (;;) {
    //check wifi status
    if (WiFi.status() != WL_CONNECTED) {
      initWiFi();
    }

    //check mqtt connetc status
    if (!pubSubClient.connected()) {
      connectMQTTServer();
    }

    //mqtt runloop
    pubSubClient.loop();

    appHeartbeat();

    vTaskDelay(100);
  }
}

void bleScanTask(void *pvParameters) {
  //setup
  initBLEScan();

  //loop
  for (;;) {
    
    bleScanTaskRunningTime = getCurrentSecTime();
    vTaskDelay(BLE_SCAN_DELAY);
    bleScanTaskRunningTime = getCurrentSecTime();
    digitalWrite(LED, HIGH);
    startBLEScan();
    digitalWrite(LED, LOW);
  }
}

void loop() {
}

/**
   initData
*/
void initData() {
  networkTaskRunningTime = 0;
  bleScanTaskRunningTime = 0;

  byte wifiMac[6];
  WiFi.macAddress(wifiMac);
  for (int i = 0;i < sizeof(wifiMac);i++){
    sprintf(MAC_char_STA,"%s%02x",MAC_char_STA,wifiMac[i]);
  }
  
  userDeviceInfos = new UserDeviceInfo[irkListLength];

  for (int i = 0; i < irkListLength; i++) {
    userDeviceInfos[i].userName = users[i];
    for (int j = 0; j < ESP_BT_OCTET16_LEN; j++) {
      userDeviceInfos[i].irk[j] = irk[i][j];
    }
  }
}

/**
   initWiFI
*/
void initWiFi() {

  WiFi.begin(WiFiSSID, WiFiPWD);

  if (WiFi.status() == WL_NO_SHIELD) {
    PRINTLN("[initWiFi]WiFi shield not present");
    while (true);
  }

  PRINTLN("[initWiFi]Connecting to AP ...");

  PRINT("[initWiFi]Attempting to connect to WPA SSID:");
  PRINT(WiFiSSID);

  long startTime = millis();
  for(;;){
    if (WiFi.status() == WL_CONNECTED){
      PRINTLN();
      PRINT("[initWiFi]Connected to AP");
      PRINTLN(WiFiSSID);
      PRINT("[initWiFi]IP address:");
      PRINTLN(WiFi.localIP());
      break;
    }else{
      if (millis() - startTime >= WIFI_CONNECT_TIMEOUT*1000){//wifi connect timeout reset boot
        resetFunc();//reset
        break;
      }else{
        PRINT(".");
      }
    }
  }
  while (WiFi.status() != WL_CONNECTED) {
    PRINT(".");
    delay(500);
  }
  

}

/**
   initMQTT
*/
void initMQTT() {
  PRINTLN("[initMQTT]Init MQTT client...");
  pubSubClient.setServer(MQTT_SERVER, MQTT_PORT);
  pubSubClient.setCallback(pubSubClientCallback);
  connectMQTTServer();
}

/**
   initBLEScan
*/
void initBLEScan() {
  PRINTLN("[initBLEScan]Init BLEScan...");
  BLEDevice::init("");
  trackerBLEScan = BLEDevice::getScan();
  // trackerBLEScan->setAdvertisedDeviceCallbacks(new TrackerAdvertisedDeviceCallbacks());
  trackerBLEScan->setActiveScan(true);
  trackerBLEScan->setInterval(100);
  trackerBLEScan->setWindow(99);
}

/**
   connectMQTTServer
*/
void connectMQTTServer() {
  while (!pubSubClient.connected()) {
    PRINTLN("[connectMQTTServer]Connecting to MQTT Server ...");
//    "TrackerI:"
    if (pubSubClient.connect(MAC_char_STA, MQTT_USER, MQTT_PWD)) {
      PRINTLN("[connectMQTTServer]MQTT service connect success");
      subscribeTopic();
    } else {
      PRINT("[connectMQTTServer]MQTT service connect fail:");
      PRINT(pubSubClient.state());
      PRINTLN(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

/**
   subscribeTopic
*/
void subscribeTopic() {
  pubSubClient.subscribe(MQTT_TOPIC_ACTION);
  pubSubClient.publish(MQTT_TOPIC_ACTION, "{\"code\":0,\"action\":0}");
}

/**
   pubSubClientCallback
*/
void pubSubClientCallback(char*topic, byte* payload, unsigned int length) {
  PRINTLN(topic);
  for (int i = 0; i < length; i++)
  {
    PRINT((char)payload[i]);
  }
  PRINTLN();
  PRINTLN("-----------------------");
}

/**
   startBLEScan
*/
void startBLEScan() {
  PRINTLN("[startBLEScan] start Scan...");
  BLEScanResults foundDevices = trackerBLEScan->start(BLE_SCAN_TIME, false);
  int deviceCount = foundDevices.getCount();

  PRINTF("[startBLEScan] scan finish device count-->%d \n", deviceCount);
  int scanFlag[irkListLength] = {0};

  for (int i = 0; i < deviceCount; i++) {
    BLEAdvertisedDevice bleAdvertisedDevice = foundDevices.getDevice(i);
    for (int j = 0; j < irkListLength; j++) {
      BLEAddress bleAddress = bleAdvertisedDevice.getAddress();
      if (btm_ble_addr_resolvable(*(bleAddress.getNative()), userDeviceInfos[j].irk)) {
        scanFlag[j] = 1;
        break;
      }
    }
  }
  scanResultHandle(scanFlag);


  trackerBLEScan->clearResults();
}


/**
   scanResultHandle
*/
void scanResultHandle(int scanResultArr[]) {

  PRINTLN("[scanResultHandle]start process...");
  bool isInit = false;
  DynamicJsonDocument jsDoc(1024);
  for (int i = 0; i < irkListLength; i++) {
    UserDeviceInfo* userDeviceInfo = &userDeviceInfos[i];

    JsonObject jbUser = jsDoc.createNestedObject(userDeviceInfo->userName);

    int originState = userDeviceInfo->currentState;
    if (!isInit && -1 == userDeviceInfo->currentState) {
      isInit = true;
    }
    if (scanResultArr[i] == userDeviceInfo->currentState) {
      userDeviceInfo->targetStateChangeCount = 0;
    } else {
      userDeviceInfo->targetStateChangeCount ++;
      if (userDeviceInfo->targetStateChangeCount >= USER_STATE_CHANGE_MAX_COUNT) {
        PRINTF("[scanResultHandle]user %s has change \n", userDeviceInfos->userName);
        userDeviceInfo->currentState = scanResultArr[i];
        userDeviceInfo->targetStateChangeCount = 0;
      }
    }
    jbUser["currentState"] = userDeviceInfo->currentState;
    jbUser["originalState"] = originState;

    PRINTF("[scanResultHandle]%s-->%d\n", userDeviceInfo->userName, userDeviceInfo->currentState);
    // PRINT(userDeviceInfo->userName);
    // PRINT("-->");
    // PRINTLN(userDeviceInfo->currentState);
    //PRINT(scanResultArr[i]);
  }
  String userStateJsStr;
  serializeJson(jsDoc, userStateJsStr);
  PRINTLN(userStateJsStr);
  jsDoc.clear();
  if (!isInit) {
    //send message
    PRINTLN("[scanResultHandle]send scan result...");
    pushMQTTMsg(MQTT_TOPIC_STATE,userStateJsStr.c_str());
    PRINTLN("[scanResultHandle]send scan result finish");
  }
  PRINTLN("[scanResultHandle]handle finished!");
}


/**
 * pushMQTTMsg
 */
void pushMQTTMsg(const char* topic, const char* payload){
  if (WiFi.status() == WL_CONNECTED && pubSubClient.connected()){
    pubSubClient.publish(topic, payload);
  }
}

/**
 * appHeartbeat
 */
void appHeartbeat() {
  long currentTime = getCurrentSecTime();
  if (currentTime - networkTaskRunningTime >= HEARTBEAT_DELAY) {

    DynamicJsonDocument jsDoc(1024);
    jsDoc["code"] = 0;
    jsDoc["networkRunntime"] = currentTime;
    jsDoc["BLEScanRunntime"] = bleScanTaskRunningTime;
    String heartbeatJSStr;
    serializeJson(jsDoc, heartbeatJSStr);
    pushMQTTMsg(MQTT_TOPIC_HEARBEAT,heartbeatJSStr.c_str());
    jsDoc.clear();

    // std::stringstream fmt;
    // fmt << "{\"code\":0,\"runntime\":" << currentTime << "}";
    // PRINTLN(fmt.str().c_str());
    // pubSubClient.publish(MQTT_TOPIC_ACTION, fmt.str().c_str());
    networkTaskRunningTime = currentTime;
  }

  currentTime = getCurrentSecTime();
  if (currentTime - bleScanTaskRunningTime >= BLESCANTASKTIMEOUT) {
    PRINTLN("ble scan task maybe dead restart!");
    bleScanTaskRunningTime = currentTime;
    //newBLEScanTask();
    resetFunc();//reset
  }

}

/**
 * getCurrentSecTime
 */
long getCurrentSecTime(){
  return millis()/1000;
}
