# Lora Remote Control

Definition of commands to be sent to my Lora based sensor nodes. To keep messages short, byte definitions are unsigned

## Byte 0 - select command group
* 0x00 - system commands
* 0x01 - sensor commands


## Commands by group
### System commands - (Byte 0 == 0x00)
* 0x01 - Power save (default on). Argument switches it on or off. So `0x01 0x00` is power save off, while `0x01 0x01` is power safe on.
* 0x02 - Delay time. Argument is delay in seconds. So `0x02 0x3c` is 60 seconds delay/sleep between measurements. `0x02 0x00` is variable delay (default)
* 0x03 - LED usage.
  * 0x03 0x00 - LED off
  * 0x03 0x01 - LED on
  * 0x03 0xff - LED shows status (default)
* 0x99 - reboot. Reboots the node (if possible)

### Sensor commands - (Byte 0 == 0x01)
* 0xff - rediscover sensors. Rediscovers and re-initializes all sensors.
* 0x1x - configure sensor x (see below for list)
  * 0x11 - Configure BME280
    * 0x11 0x01 - stop temperature reading
    * 0x11 0x02 - stop humidity reading
    * 0x11 0x04 - stop pressure reading
    * 0x11 0x07 - stop all readings (combined three bits together)
    * 0x11 0x08 - restart all readings
