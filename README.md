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

### I2C1 (DFR GNSS Module)
| GNSS Pin | Chip Pin | Function |
| -------- | --------- | ------- |
| D/T | PF_0 | I2C1_SCL |
| G/R | PF_1 | I2C1_SDA |

### 8-bit Serial (3.5" LCD Display)
| LCD Pin | Chip Pin | User Label |
| ------- | -------- | ---------- |
| D0 | PG_9 | D0 |
| D1 | PG_14 | D1 |
| D2 | PF_15 | D2 |
| D3 | PE_13 | D3 |
| D4 | PF_14 | D4 |
| D5 | PE_11 | D5 |
| D6 | PE_9 | D6 |
| D7 | PF_13 | D7 |
| RD | PB_1 | RD |
| WR | PC_2 | WR |
| RS | PF_4 | RS |
| CS | PB_6 | CS |
| RST | PB_2 | RST |


## References
Lora
- [STM32 LoRa Repo](https://github.com/SMotlaq/LoRa)
- [Adafruit LoRa Pinout](https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/pinouts)
- [Adafruit LoRa Guide](https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/using-the-rfm69-radio)
- [RadioHead Repo](https://github.com/adafruit/RadioHead/tree/master)

GNSS
- [DFRobot GNSS Guide](https://wiki.dfrobot.com/dfr1103/)
- [DFRobot Repo](https://github.com/DFRobot/DFRobot_GNSS/tree/master)

3.5" LCD Display
- [How to use The TFT LCD Touch Screen Module for Arduino](https://learn.carobot.ca/guide/touch-screen-module)

STM32
- [ST-Nucleo-F446ZE Pinout](https://os.mbed.com/platforms/ST-Nucleo-F446ZE/)
- [ST Wiki](https://wiki.st.com/stm32mcu/wiki/STM32StepByStep:Step1_Tools_installation)
