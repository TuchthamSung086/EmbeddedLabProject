# EmbeddedLabProject
 Mixed Blood at it again

Our realtime database Firebase link: https://embedded-lab-project-default-rtdb.asia-southeast1.firebasedatabase.app/

## Useful Resource
### Youtube videos
1. https://youtu.be/EsCkSNrWyq8 : Send and read data to Firebase using esp8266 nodemcu wifi module (2021) // I follow this mainly, use Arduino IDE
2. https://youtu.be/ZZ5JvSA-Ed8 : SOLUTION] Connection To Google Firebase Real Time Database Failed NodeMCU ESP8266 // Lucky I don't encounter this problem... YET
3. https://www.youtube.com/watch?v=vcXQ79YxeGM : Arduino serial communication using software serial library // We don't use this, but peeps said it's nice
4. https://youtu.be/LS7owD8C_tI : ESP8266 WebServer using STM32 HAL|| LED Control || Ring Buffer // Maybe we can use STM32 alone... but it's already working with Arduino IDE so I'd like to keep it that way.

### Blog Post
- https://controllerstech.com/create-1-microsecond-delay-stm32/: Microsecond delay library
- https://controllerstech.com/using-dht11-sensor-with-stm32/ : DHT11 library
- https://maker.pro/arduino/projects/arduino-uno-sharp-dust-sensor-tutorial : Dust sensor code
- https://controllerstech.com/ir-remote-with-stm32/ : Explain air Conditoner IR protocol, TSOP sensor, pesudocode sample

## Our equipments
Touchy: USB to mini-USB wire (very important, got it from a friend AKA SirateeK) , ESP8299 V3, STM32 Board

Pooh: ESP8266 NodeMCU V3, STM32F4xx Board, Breadboard, Sensors, Wires, lot of wire

### Sensor
|Name|Measure|
|-|-|
|DHT11|Temperature and humidity|
|LDR|Brightness|
|TSOP4838|38KHz IR signal|
|GP2Y1010AU0F|Dust density|

## Encountered problem/solution
- Timer not start: HAL_TIM_BASE_START_IT
- DHT give very high/low value: DHT need to be powered on for atleast 3 second before measuring, eg. 3s after upload/reset
- Esp8266 appered on lsusb but not in /dev/tty* on linux: remove/disable brltty service