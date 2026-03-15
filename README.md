## Block Diagram
![342 Project Block Diagram.drawio.png](https://github.com/mogggit/peter-long-range/blob/main/342%20Project%20Block%20Diagram.drawio.png?raw=true)

## Pinout & Configuration
### SPI1 (RFM95W LoRa)
Mode: Full-Duplex Master<br>
<br>
| LoRa Pin | Chip Pin | Function |
| -------- | --------- | -------- |
| SCK | PA_5 | SPI1_SCK |
| MISO | PA_6 | SPI1_MISO |
| MOSI | PA_7 | SPI1_MOSI |
| CS | PA_4 | SPI1_NSS |
| G0 | PF_12 | GPIO_EXTI12 |
| RST | PB_4 | RESET_GPIO_Port |


## References
Lora
- [Adafruit LoRa Guide](https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/using-the-rfm69-radio)
- [RadioHead Repo](https://github.com/adafruit/RadioHead/tree/master)

GNSS
- [DFRobot GNSS Guide](https://wiki.dfrobot.com/tel0157/docs/22215)
- [DFRobot Repo](https://github.com/DFRobot/DFRobot_GNSS/tree/master)

STM32
- [ST Wiki](https://wiki.st.com/stm32mcu/wiki/STM32StepByStep:Step1_Tools_installation)
