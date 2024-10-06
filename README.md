# home-assistant-modbus-ecactus-ecos
## Let home assistant control your all in one home battery via modbus and esphome.

This yaml can be used to control ecactus-ecos systems.
The main purpose is to make the BMS (battery management system) available in Home Assistant to make automated charge/discharge possible. 
Use at your own risk.

## Installing ESPHome Manually
* ```python --version``` Should give Python vesion 3.xx.x if not install python
* ```pip3 install wheel```
* ```pip3 install esphome```
* ```esphome version``` should return Version: 202x.x.x.
* Edit the secrets.yaml to reflect your local settings
* ```esphome run ecactus.yaml``` on the first time you have to connect usb to pc. All other version can go over the air as long you are connected to same network 
 
# some remarks: 
* Make sure that your wifi credentials are set and asign a ip address,gateway and subnetmask in the yaml.
* The yaml is tested on a esp32 connected to a TTL to RS485 converter. 
* BMS settings can be selected in home assistant via list options and are pushed when [change modus] button is pressed.
* Known issue:  Component modbus_controller took a long time for an operation (0.05 s) and Components should block for at most 20-30ms i seen no effect or solution.
* Please be kind this is the first yaml i have ever written :-)

# hardware used:
   * ecactus-ecos
   * ESP32-S3
   * TTL to RS485 converter
   * container
   * some wire

# setup home assistant with: 
  * HACS
  * ESPHome
  * Power Flow Card

# setup 
  * Flash ESPHome to ESP32
  * Edit yaml on ESP32
 
 # Hardware setup
 
 ![setup](schema.png)

# lovelace setup

```
 type: custom:power-flow-card
watt_threshold: 1000
entities:
  battery: sensor.yoursensorname_dsp_power
  battery_charge: sensor.yoursensorname_esp_v2_soc
  grid: sensor.yoursensorname_grid_power
  solar: sensor.yoursnesorname_pv_power
```





 