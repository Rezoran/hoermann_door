# hoermann_door
Control Hörmann doors drives directly via MQTT from Home Assistant

Inspired by 
- https://blog.bouni.de/posts/2018/hoerrmann-uap1/
- https://github.com/14yannick/hoermann_door
- https://github.com/steff393/hgdo
- https://github.com/avshrs/ESP32_Hormann_Supramatic_e3
- https://patents.google.com/patent/WO2005076529A1/de

**USE AT YOUR OWN RISK!**

# Overview

This repo contains the software in form of an esphome component which connects Hörmann door drives like the Supramatic E3 to Home Assistant. The esp emulates an UAP1 to interact with the door drive.
**NOT compatible with drives from the 4 series**

# Hardware used
## My setup
- esp32 devkit 4
- https://www.ti.com/product/SN65HVD72 off a salvaged board
- DC/DC 24V -> 5V Step-Down
## Possible working setups
I haven't tested it, but boards from the 4 Series drives that only feature and RS485-UART converter should theoretically work (rts-pin has to be undefined)
Otherwise a RS485 Transceiver breakout board should also work 
