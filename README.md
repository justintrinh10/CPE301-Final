**CPE 301 Embedded Systems Design \- Final Project**

Final Project for CPE 301 (Fall 2025\)

**Group Member**

Xuerou Cai, Shawn Meng, Jacklyn Trinh, Justin Trinh

**Project Description**  
This project is an evaporative cooling system (“swamp cooler”) built using the Arduino Mega 2560\.    
The system monitors water level, air temperature, and humidity, displays data on an LCD screen,    
controls a fan motor based on temperature, adjusts a vent using a stepper motor, and records    
state transitions using an RTC module.

The cooler implements the required state machine:  
\- DISABLED    
\- IDLE    
\- RUNNING    
\- ERROR  

**Components Used**  
\- Arduino Mega 2560  
\- DHT11 Temperature & Humidity Sensor  
\- Water Level Sensor  
\- LCD1602 Display (I2C)  
\- Stepper Motor \+ Driver  
\- Fan Motor \+ External Power Board  
\- Real-Time Clock Module (DS1307)  
\- Push Buttons (Start / Stop / Reset)  
\- LEDs (Yellow, Green, Red, Blue)

**Features**  
\- Real-time monitoring of temperature & humidity (displayed on LCD)  
\- Water level detection (transitions to ERROR when too low)  
\- Fan motor control based on temperature thresholds  
\- Stepper motor vent angle adjustment  
\- System enable/disable using ISR-based Start button  
\- Error state handling with reset capability  
\- State transitions logged with timestamps via serial output

**Cooler State Machine**  
\- DISABLED: System off, yellow LED on, waiting for START button (interrupt)  
\- IDLE: Green LED on, monitoring sensors, transitions based on temperature/water level  
\- RUNNING: Blue LED on, fan on, returns to IDLE when temperature drops  
\- ERROR: Red LED on, fan off, requires reset when water returns to safe level

**Circuit Diagram**  
![screenshot](Image/Circuit%201.png)

![screenshot](Image/Circuit%202.png)

![screenshot](Image/Circuit%203.png)

![screenshot](Image/Circuit%203.png)


**System Pictures**  

![screenshot](Image/Code%201.png)

![screenshot](Image/Code%202.png)

![screenshot](Image/Code%203.png)


**Demonstration Video**  **Demonstration Video**  
Video link: https://youtu.be/t3BCde3PAho



