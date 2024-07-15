# Smart-Home-Automation-Project-with-Esp-Rainmaker-Platform

## Smart Home Automation with voice control and sensors Integration
This project involves creating a smart home automation system using ESP32 with ESP Rain Maker IoT platform. 
Controlling 4 relays via app, manual switches, and voice assistants (Google, Alexa, Siri). 
Includes temperature/humidity, and light sensors for automated control.
Integrates with Google Home, HomeKit, and Amazon Alexa.

Another device with a motion sensor is also added for security, which detects motion, sends notifications, and turns on a buzzer. The sensor can be turned on/off through the app, making it useful when required, mainly during nighttime.

---

## Table of Contents

- [Introduction](#introduction)
- [Purpose of the Project](#purpose-of-the-project)
- [Hardware Required](#hardware-required)
- [Software Required](#software-required)
- [Libraries Required](#libraries-required)
- [Circuit Diagram](#circuit-diagram)
- [C++ and OOP Concept Approach in Project Source Code](#c++-and-oop-concept-approach-in-project-source-code)
- [C++ Concepts Utilized in the Code](#c++-concepts-utilized-in-the-code)
- [Code Explanation](#code-explanation)
- [Setup and Usage](#setup-and-usage)


---

## Introduction

This project demonstrates a smart home automation system using the ESP32 microcontroller and ESP Rain Maker to control 4 appliances with Google Assistant, Alexa, and manual switches. The system allows for remote control and monitoring of various appliances and sensor readings from anywhere in the world through a mobile app or web interface. It also includes manual control and sensor integration for automated responses based on environmental conditions.

## Purpose of the Project

The purpose of this project is to create a smart home automation system using the ESP32 microcontroller and the ESP RainMaker platform. This project is designed as part of the C++ coursework for the 3rd semester at KEC. The primary aim is to implement Object-Oriented Programming (OOP) concepts to build a functional and practical IoT solution that can control and automate household appliances, thus enhancing daily convenience and efficiency.


## Hardware Required

- ESP32 Development Board
- Relays (4)
- DHT11/DHT22 Temperature and Humidity Sensor
- LDR (Light Dependent Resistor)
- Push Buttons (4)
- LEDs (Optional for indicators)
- Resistors (as needed)
- Connecting Wires

## Software Required

- Arduino IDE


## Libraries Required

To run the code, you need to install the following libraries in the Arduino IDE:

1. **WiFi**: For connecting the ESP32 to WiFi.
2. **WiFiProv**: For provisioning WiFi credentials.
3. **RMaker**: For integrating with ESP Rain Maker.
4. **DHT**: For reading data from the DHT11/DHT22 sensor.
5. **SimpleTimer**: For handling timed operations.

You can install these libraries through the Library Manager in the Arduino IDE.

## Circuit Diagram

Connect the components as follows:

- **Relays**: Connect the control pins of the relays to GPIO pins 23, 22, 21, and 19.
- **Switches**: Connect one terminal of each switch to GPIO pins 13, 12, 14, and 27, and the other terminal to ground.
- **DHT Sensor**: Connect the data pin to GPIO 18.
- **LDR**: Connect one terminal to GPIO 39 (Analog pin), and the other terminal to 3.3V through a resistor.

Refer to the ESP32 pinout for exact pin locations.

## Code Explanation

The provided code is organized into several sections:

### 1. Initialization and Setup

The code starts by including necessary libraries and defining constants for device names, pin numbers, and other parameters. It initializes the DHT sensor and sets up the GPIO pins.

```cpp
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <DHT.h>
#include <SimpleTimer.h>

const char *service_name = "CY_SmartHome";
const char *pop = "12345678";

// Define the Chip Id
uint32_t espChipId = 5;

// Define the Node Name
char nodeName[] = "CY_Smarthome";

// Define the Device Names
char deviceName_1[] = "Light";
char deviceName_2[] = "Night Light";
char deviceName_3[] = "Fan";
char deviceName_4[] = "Power Socket";

// Define the GPIO connected with Relays and switches
static uint8_t RelayPin1 = 23;
static uint8_t RelayPin2 = 22;
static uint8_t RelayPin3 = 21;
static uint8_t RelayPin4 = 19;

static uint8_t SwitchPin1 = 13;
static uint8_t SwitchPin2 = 12;
static uint8_t SwitchPin3 = 14;
static uint8_t SwitchPin4 = 27;

static uint8_t wifiLed = 2;
static uint8_t gpio_reset = 0;
static uint8_t DHTPIN = 18;
static uint8_t LDR_PIN = 39;

float temperature1 = 0;
float humidity1 = 0;
float ldrVal = 0;

DHT dht(DHTPIN, DHT11);
SimpleTimer Timer;
```

### 2. Provisioning and Callbacks
The sysProvEvent and write_callback functions handle provisioning events and actions for the smart devices.

```cpp
void sysProvEvent(arduino_event_t *sys_event) {
  // Handle provisioning events
}

void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx) {
  // Handle device actions
}
```
### 3. Reading Sensors
The readSensor and sendSensor functions read data from the DHT sensor and LDR, and send the data to ESP Rain Maker.

```cpp
void readSensor() {
  // Read data from sensors
}

void sendSensor() {
  readSensor();
  temperature.updateAndReportParam("Temperature", temperature1);
  humidity.updateAndReportParam("Temperature", humidity1);
  ldr.updateAndReportParam("Temperature", ldrVal);
}
```
### 4. Manual Control
The manual_control function handles manual switching of devices using physical buttons.

```cpp
void manual_control() {
  // Check switch states and control relays
}
```
### 5. Setup and Loop
The setup function initializes the ESP32, sets up devices, and starts provisioning. The loop function handles periodic sensor reading and manual control.

```cpp

void setup() {
  // Initialization and setup code
}

void loop() {
  // Main loop code
}
```


## C++ and OOP Concept Approach in Project Source Code

In the provided source code for the Smart Home Automation project using ESP32 Rain Maker, C++ and Object-Oriented Programming (OOP) concepts are effectively utilized to structure and manage the functionality of the system. Here’s how C++ and OOP principles are applied:

### Object-Oriented Approach

#### Classes and Objects
- **Switch Class**: Defined for managing relay-based switches (e.g., `my_switch1`, `my_switch2`, etc.). Each switch is instantiated as an object (`Switch`) with methods to control its state (`toggleState`) and update parameters (`updateAndReportParam`).
- **TemperatureSensor, HumiditySensor, LDR Classes**: These classes encapsulate sensor functionalities (temperature, humidity, LDR). They provide methods to read sensor values (`readSensor`), update parameters (`updateAndReportParam`), and handle sensor-specific operations.

#### Encapsulation
- Member variables (`toggleState_1`, `toggleState_2`, etc.) and member functions (`write_callback`, `readSensor`, `sendSensor`, `manual_control`) are encapsulated within their respective classes. For instance:
  - The **Switch** class encapsulates relay control logic and callback handling (`write_callback`).
  - **Sensor** classes encapsulate sensor reading logic (`readSensor`) and parameter reporting (`updateAndReportParam`).

#### Abstraction
- Abstracting device functionalities into classes (**Switch**, **TemperatureSensor**, etc.) allows treating each device type uniformly while hiding implementation details. This promotes code reuse, enhances modularity, and simplifies maintenance.

### C++ Concepts Utilized in the Code

1. **Libraries and Includes**
    - `#include "RMaker.h"`: Includes the ESP RainMaker library for cloud connectivity and device provisioning.
    - `#include "WiFi.h"`: Includes the Wi-Fi library for ESP32.
    - `#include "WiFiProv.h"`: Includes the Wi-Fi provisioning library.
    - `#include <DHT.h>`: Includes the library for DHT temperature and humidity sensors.
    - `#include <SimpleTimer.h>`: Includes the SimpleTimer library for managing timed events.

2. **Global Constants and Variables**
    - **Constants**: Defined for service name, PoP, chip ID, node name, and device names.
    - **GPIO Pins**: Constants defined for relay pins, switch pins, WiFi LED, reset pin, DHT pin, and LDR pin.
    - **States**: Boolean variables to track the state of relays and switches.
    - **Sensor Values**: Variables to store temperature, humidity, and LDR values.

3. **Object Instantiation**
    - **DHT Sensor**: `DHT dht(DHTPIN, DHT11);` creates an instance for the DHT11 sensor.
    - **SimpleTimer**: `SimpleTimer Timer;` creates a timer instance.
    - **Device Instances**: Objects for switches and sensors using the RainMaker framework.

4. **Functions**
    - `sysProvEvent()`: Handles system provisioning events and updates Wi-Fi LED status.
    - `write_callback()`: Callback function for handling device parameter updates.
    - `readSensor()`: Reads data from the DHT sensor and LDR.
    - `sendSensor()`: Sends sensor data to the cloud.
    - `manual_control()`: Handles manual control of devices via switches.

5. **Setup and Loop Functions**
    - `setup()`: Initializes serial communication, sets up GPIOs, initializes the DHT sensor, sets up RainMaker nodes and devices, and starts Wi-Fi provisioning.
    - `loop()`: Monitors the reset button, manages Wi-Fi connectivity, sends sensor data at intervals, and handles manual control.

6. **Detailed C++ Concepts**
    - **Object-Oriented Programming (OOP)**
        - **Class Instantiation**: Instances of DHT, SimpleTimer, and RainMaker devices.
        - **Method Calls**: Using object methods like `dht.readHumidity()`, `Timer.setInterval()`, and `RMaker.initNode()`.
        - **Callback Functions**: Implementing callbacks such as `write_callback()` for handling device events.
    - **Control Structures**
        - **Conditionals**: `if` statements to handle different conditions in `write_callback()`, `manual_control()`, and `loop()`.
        - **Loops**: `for` loop in `setup()` for generating the chip ID; `while` loop in `loop()` for button debounce.
    - **Data Types and Operators**
        - **Data Types**: Usage of `const char*`, `uint8_t`, `bool`, `float`, `int`.
        - **Operators**: Arithmetic (`+`, `-`), logical (`&&`, `||`), comparison (`==`, `!=`), assignment (`=`, `+=`), and ternary (`?:`) operators.
    - **GPIO Manipulation**
        - `pinMode()`: Configuring GPIOs as INPUT or OUTPUT.
        - `digitalWrite()`: Setting GPIO states.
        - `digitalRead()`: Reading GPIO states.
    - **Serial Communication**
        - `Serial.begin()`: Initializing serial communication.
        - `Serial.printf()`: Printing formatted strings for debugging and status updates.
    - **ESP32 Specific Functions**
        - `ESP.getEfuseMac()`: Reading the ESP32 MAC address.
        - `WiFi.onEvent()`: Registering Wi-Fi event handlers.
        - `WiFiProv.beginProvision()`: Starting Wi-Fi provisioning.


### Setup and Usage

Hardware Setup: Connect all components as per the circuit diagram.
Software Setup: Install the required libraries in the Arduino IDE.
Provisioning: Use the ESP Rain Maker app to provision the device and set up WiFi credentials.

### Control Appliances with Google Assistant and Alexa

You can control the appliances using voice commands with Google Assistant and Alexa. For example:
- "Hey Google, turn on switch-1" will turn on the light connected to switch-1.
- "Hey Google, turn off switch-1 and switch-3" will turn off the lights connected to switch-1 and switch-3.
- "Hey Google, turn on switch 2 and switch 4" will turn on the lights connected to switch-2 and switch-4.
- "Hey Google, turn on all switches" will turn on all connected lights.

If you have any Alexa or Google Assistant-supported devices, you can control appliances from them directly. For example, using a smartwatch with built-in Alexa support, you can say "Turn On switch 1" to control the appliances.

### Control Appliances with Manual Switches

You can control the lights using manual switches even without an internet connection. When Wi-Fi is turned off, the blue LED will turn off, indicating that only manual control is available. When Wi-Fi is restored, the ESP32 will automatically reconnect, and the blue LED will turn on, allowing control via Alexa and Google Assistant again.

### Monitoring and Automation with ESP RainMaker

Using the ESP RainMaker app, you can control switches and monitor sensor readings for temperature, humidity, and light levels. For instance:
- Covering the LDR sensor will decrease its value.
- Increasing the temperature with external heat will show the corresponding increase in sensor readings.

You can also add automation to control the relays based on a predefined schedule. For example, you can create a schedule to turn on the 2nd and 4th relays at a specific time. This feature allows for highly customizable automation according to your requirements.



