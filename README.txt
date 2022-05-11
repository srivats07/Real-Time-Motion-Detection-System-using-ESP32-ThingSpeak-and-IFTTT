# ECE 568 - Lab Assignment - 3

motion_detector.py:
1. Connects ESP32 device to active wireless network.
2. Calibrates Adafruit MPU6050 Accelerometer sensor everytime the system boots up.
3. Constantly listens to ThingSpeak server for predefined keywords from the User every 30 seconds.
4. The user uses Google Assistant to sent keywords to the Thingspeak server.
5. If User keyword is "activate", the system is said to be in "ON" state (Active Motion-Detection). 
The program turns on Green Led and calculates accelerometer sensor values.
6. In "ON" state, the device monitors whether accelerometer values. If the accelerometer values exceeds threshold, 
Red Led is turned on and notification is sent to the User via IFTTT app.
7. If User keyword is "deactivate", the program turns off Green Led and stops calculating accelerometer values.
And the system is said to be in "OFF" state (No Motion-Detection).

Hardware Connection details:
* Communication between MPU6050 and ESP32 is established using I2C Protocol.
* SCL Pin of MPU6050 is connected to GPIO Pin 22 of ESP32 Featherboard.
* SDA Pin of MPU6050 is connected to GPIO Pin 23 of ESP32 Featherboard.
* GPIO Pin 12 of ESP32 Featherboard is connected to the positive terminal of green led, and the negative terminal
of green led is connected to the ground via a 220 ohms resistor.
* GPIO Pin 13 of ESP32 Featherboard is connected to the positive terminal of red led, and the negative terminal
of red led is connected to the ground via a 220 ohms resistor.


Video Link: 
https://youtu.be/-uYP5EsYDrk

 

