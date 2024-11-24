# MOVED
**This project has been merged into https://github.com/14yannick/hoermann_door**
<br><br><br><br><br><br><br><br><br><br><br><br><br>
# hoermann_door
Control Hörmann doors drives directly via MQTT from Home Assistant - Without custom PCB

Inspired by 
- https://blog.bouni.de/posts/2018/hoerrmann-uap1/
- https://github.com/14yannick/hoermann_door
- https://github.com/steff393/hgdo
- https://github.com/avshrs/ESP32_Hormann_Supramatic_e3
- https://patents.google.com/patent/WO2005076529A1/de

**USE AT YOUR OWN RISK!**

# Overview

This repo contains the software in form of an esphome component which connects Hörmann door drives like the Supramatic E3 to Home Assistant. The esp emulates an UAP1 to interact with the door drive.

**NOT compatible with drives from the 4th series**

<p align="center">
    <img src="docs/PCB.png?raw=false" width="400px">
    <img src="docs/installation.png?raw=false" width="230px">
</p>

# Hardware used
## My setup
- esp32 devkit 4
- DC/DC 24V -> 5V Step-Down
- https://www.ti.com/product/SN65HVD72 off a salvaged board

OR
- RS485-UART Coverter HW519
## possible working hardware
The custom boards for the Series 4 drives should also work. *But the pinout is different!*

See [docs/hoermann.md](docs/hoermann.md) for the pinout
