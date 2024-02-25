# LVGL-BIKE

![Arduino](https://img.shields.io/badge/-Arduino-00979D?style=for-the-badge&logo=Arduino&logoColor=white) ![Espressif](https://img.shields.io/badge/espressif-E7352C.svg?style=for-the-badge&logo=espressif&logoColor=white)

*This is NO-NAV version. Here is no navigation system, but other features are present (I guess)!*

LVGL-BIKE (not final project name) is a project that replicates bike computers with nice interface by lvgl and endless modifications and improvments by esp32. 

### **What do you need:**
* Esp32 (no matter which devkit, I am using DOIT)
* Ili9341 display 320x240 WITH TOUCH (it is mandatory for now because there is no support for button controls)
* Any light controlled by a relay
* Hall sensor, for example KY-003

### Assembling
Let's start with the display (ILI9341):
| Display pin | ESP32 pin |
|-------------|----------:|
| T_IRQ       | Not connected|
| T_D0        | D19       |
| T_DIN       | D23       |
| T_CS        | D21       |
| T_CLK       | D18       |
| SD0 (MISO)  | Not Connected|
| LED         | 3v3       |
| SCK         | D18       |
| SDI (MOSI)  | D23       |
| DC          | D2        |
| RESET       | D4        |
| CS          | D15       |
| GND         | GND       |
| VCC         | 3v3       |

Then, the first light:

| Relay pin | ESP32 pin |
|-----------|----------:|
| +         | Vin (5v)  |
| -         | GND       |
| S (signal)| D13       |

And finally the hall sensor:

| Sensor pin | ESP32 pin |
|------------|----------:|
| +          | 3v3       |
| -          | GND       |
| S (signal) | D12       |

In future there will be battery control, maybe navigation and bluetooth/mp3 playing.
