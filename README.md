# esp_ble_tracker

> Track the device according to the preset IRK by scanning the MAC address of the BLE device, and send status change notifications via MQTT

# How to use

- Burn the "get_irk" to the ESP32, use the phone's Bluetooth to pair with the ESP32, and print the IRK of the phone's Bluetooth in the console.

  （If you have multiple devices, please repeat the above steps）

- Modify the macro definition in the "tracker/TrackerI.ino".

  eg:`WiFiSSID`,`WiFiPWD`,`MQTT_SERVER`,`MQTT_USER`,`MQTT_PWD`

- Enter your device's IRK and username in the "tracker/TrackerI.ino"

  ```c++
  uint8_t irk[][ESP_BT_OCTET16_LEN] = {
    //your irks
    //eg:
    //{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    //{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
  };
  ```

  ``` c++
  String users[] = {
    //users
    //eg:
    "Jack", 
    "LeeMing"
  };
  ```

- Compile and burn "tracker/TrackerI.ino" to ESP32 and power up

- Subscribe to the MQTT topic "v1/trackeri/state" and you will receive status messages

  eg:

  ```json
  {
      "Jacky": {
          "currentState": 1,
          "originalState": 1
      },
      "Danson": {
          "currentState": 1,
          "originalState": 1
      },
      "Neely": {
          "currentState": 1,
          "originalState": 1
      }
  }
  ```

  state code -1:Init

  state code 0:Not Found

  state code 1:Found

# Integrated

> Because the MQTT message queue is used, it can theoretically be integrated into all platforms that support MQTT

- Home Assistant

  ```yaml
  sensor:
   - platform:  mqtt
     name:  "Danson"
     state_topic:  "v1/trackeri/state"
     value_template:  "{{value_json['danson']['currentState'] | replace('1','At home') | replace('0','Leave home')}}"
   - platform:  mqtt
     name:  "DansonII"
     state_topic:  "v1/trackeri/state"
     value_template:  "{{value_json['dansonII']['currentState'] | replace('1','At home') | replace('0','Leave home')}}"
   - platform:  mqtt
     name:  "Neely"
     state_topic:  "v1/trackeri/state"
     value_template:  "{{value_json['Neely']['currentState'] | replace('1','At home') | replace('0','Leave home')}}"
  ```

  

- Other

