# LVGL-BIKE

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
