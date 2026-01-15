/*
| Cause                                         | What Happens                                                                                                                               |
| --------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| 🔄 Flashing new firmware                      | If the new firmware changes how Bluetooth is initialized (e.g., different device name or MAC), the stored bond keys may not match anymore. |
| 🧽 Erasing flash (`esptool.py --erase_flash`) | The controller still thinks it’s paired, but the ESP32 lost its part of the bond.                                                          |
| 🔌 Controller resets or updates               | The PS4 controller may reset or overwrite its paired list but still tries to connect.                                                      |
| ⚡ Incomplete power cycles                     | A brown-out or power glitch may corrupt the pairing NVS data on the ESP32.                                                                 |
| 🔁 Switching between controllers              | Trying to pair multiple controllers without clearing bonds can cause confusion in the ESP32's bond list.                                   |
| 👯 Duplicate MAC spoofing                     | If two ESP32s try to use the same MAC address or name, the controller might bond to one but try to connect to the other.                   |
*/

#include <PS4Controller.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

void setup() 
{
  PS4.begin();  
  uint8_t pairedDeviceBtAddr[20][6];  
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for(int i = 0; i < count; i++) 
  {
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
}
 
void loop() 
{
}