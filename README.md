# ESPBrushlessController




## Microcontroller (ESP32)  
[link](https://www.amazon.nl/Diymore-Development-NodeMCU-Bluetooth-CH9102F/dp/B0D9LFM1MG/ref=sr_1_1_sspa)  
Pinout  
| left |  |  |  | right |
| --- | --- | --- | --- | --- |
| GND | rst |  | 01 TX | GND |
| nc | ~~36 SVP (input only)~~ |  | 03 RX | **27 (good)** |
| ~~39 SVN (input only)~~ | **26 (good)** |  | 22 SCL | **25 (good)** |
| ~~35 (input only)~~ | **18 (good)** |  | 21 SDA | **32 (good)** |
| **33 (good)** | **19 (good)** |  | **17  (good)** | **12 TD1 (good)** |
| ~~34 (input only)~~ | **23 (good)** |  | **16 (good)** | **4 (good)** |
| **14 TMS (good)** | **5 (good)** |  | GND | ~~0 (low to flash)~~ |
| nc | 3.3V |  | 5V | ~~2 (led?)~~ |
| ~~9 SD2 (internal flash)~~ | **13 TCK (good)** |  | **15 TD0 (good)** | ~~8 SD1 (internal flash)~~ |
| ~~11 CMD (internal flash)~~ | ~~10 SD3 (internal flash)~~ |  | ~~07 SD0 (internal flash)~~ | ~~6 CLK (internal flash)~~ |

  

  
## ESC (Speedybee)  
[link](https://www.speedybee.com/speedybee-bls-60a-30x30-4-in-1-esc/)  
Pinout  

| Pin | Description |
|---| --- |
| 1| GND| 
| 2| VBAT| 
| 3| Motor 1| 
| 4| Motor 2| 
| 5| Motor 3| 
| 6| Motor 4| 
| 7| Current| 
| 8| N/C | 

Type connector: ???

## Commands
Serial and I2C Interface
(default address i2c = 0x40)
`S [motor] [speed]`: Set motor speed (0-???)  
`A [rate]`: Set acceleration rate  
`D [0/1]`: Toggle demo mode  
`X`: Emergency stop  