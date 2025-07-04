# Temperature and Pump Control System

## Overview
This project implements a temperature and pump control system designed for the microcontroller M5Stack Core 2, using PlatformIO and C++. The C++ code can be found inside the "src" folder. The IDE used was Visual Studio Code.

The system reads temperature sensors and controls a peltier module to maintain desired temperature ranges as well as a pump for flow control. It is suitable for laboratory or industrial applications.

The codebase is modular, using PlatformIO’s structure for easy development and testing. 
The platformio.ini file indicates the libraries needed to run the application. 

## Features
- Temperature monitoring via sensor
- Automatic pump control based on temperature thresholds  
- Configurable through `platformio.ini`  
- Includes unit test placeholders  
- Compatible with multiple embedded hardware platforms supported by PlatformIO  

## Technologies
- C++  
- PlatformIO framework  

## Project Structure
- `src/` — Main application code (`main.cpp`)  
- `include/` — Header files and declarations  
- `lib/` — Additional libraries (empty or custom)  
- `test/` — Test folder with placeholder README  
- `.vscode/` — Editor configuration files  
- `platformio.ini` — PlatformIO project configuration  
- `.gitignore` — Git ignore rules  

### Prerequisites
- PlatformIO installed
- Compatible embedded device supported by PlatformIO (In this case the microcontroller M5Stack Core 2)
