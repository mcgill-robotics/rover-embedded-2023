# Teensy_PWM Library

[![arduino-library-badge](https://www.ardu-badge.com/badge/Teensy_PWM.svg?)](https://www.ardu-badge.com/Teensy_PWM)
[![GitHub release](https://img.shields.io/github/release/khoih-prog/Teensy_PWM.svg)](https://github.com/khoih-prog/Teensy_PWM/releases)
[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/khoih-prog/Teensy_PWM/blob/main/LICENSE)
[![contributions welcome](https://img.shields.io/badge/contributions-welcome-brightgreen.svg?style=flat)](#Contributing)
[![GitHub issues](https://img.shields.io/github/issues/khoih-prog/Teensy_PWM.svg)](http://github.com/khoih-prog/Teensy_PWM/issues)

<a href="https://www.buymeacoffee.com/khoihprog6" title="Donate to my libraries using BuyMeACoffee"><img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" alt="Donate to my libraries using BuyMeACoffee" style="height: 50px !important;width: 181px !important;" ></a>
<a href="https://www.buymeacoffee.com/khoihprog6" title="Donate to my libraries using BuyMeACoffee"><img src="https://img.shields.io/badge/buy%20me%20a%20coffee-donate-orange.svg?logo=buy-me-a-coffee&logoColor=FFDD00" style="height: 20px !important;width: 200px !important;" ></a>


---
---

## Table of Contents

* [Why do we need this Teensy_PWM library](#why-do-we-need-this-Teensy_PWM-library)
  * [Features](#features)
  * [Why using hardware-based PWM is better](#why-using-hardware-based-pwm-is-better)
  * [Currently supported Boards](#currently-supported-boards)
* [Changelog](changelog.md)
* [Prerequisites](#prerequisites)
* [Installation](#installation)
  * [Use Arduino Library Manager](#use-arduino-library-manager)
  * [Manual Install](#manual-install)
  * [VS Code & PlatformIO](#vs-code--platformio)
* [Packages' Patches](#packages-patches)
  * [1. For Teensy boards](#1-For-Teensy-boards)
* [Usage](#usage)
  * [1. Create PWM Instance with Pin, Frequency, dutycycle](#1-create-pwm-instance-with-pin-frequency-dutycycle)
  * [2. Initialize PWM Instance](#2-Initialize-PWM-Instance)
  * [3. Set or change PWM frequency or dutyCycle](#3-set-or-change-PWM-frequency-or-dutyCycle)
  * [4. Set or change PWM frequency and dutyCycle manually and efficiently in waveform creation](#4-Set-or-change-PWM-frequency-and-dutyCycle-manually-and-efficiently-in-waveform-creation)
* [Examples](#examples)
  * [ 1. PWM_Basic](examples/PWM_Basic)
  * [ 2. PWM_DynamicDutyCycle](examples/PWM_DynamicDutyCycle) 
  * [ 3. PWM_DynamicDutyCycle_Int](examples/PWM_DynamicDutyCycle_Int)
  * [ 4. PWM_DynamicFreq](examples/PWM_DynamicFreq)
  * [ 5. PWM_Multi](examples/PWM_Multi)
  * [ 6. PWM_MultiChannel](examples/PWM_MultiChannel)
  * [ 7. PWM_Waveform](examples/PWM_Waveform)
  * [ 8. PWM_StepperControl](examples/PWM_StepperControl) **New**
* [Example PWM_Multi](#example-PWM_Multi)
* [Debug Terminal Output Samples](#debug-terminal-output-samples)
  * [1. PWM_DynamicDutyCycle using FlexTimers on Teensy 4.0](#1-PWM_DynamicDutyCycle-using-FlexTimers-on-Teensy-40)
  * [2. PWM_Multi using QuadTimers on Teensy 4.0](#2-PWM_Multi-using-QuadTimers-on-Teensy-40)
  * [3. PWM_DynamicFreq using FlexTimers on Teensy 4.0](#3-PWM_DynamicFreq-using-FlexTimers-on-Teensy-40)
  * [4. PWM_Waveform using FlexTimers on Teensy 4.0](#4-PWM_Waveform-using-FlexTimers-on-Teensy-40)
  * [5. PWM_Waveform using QuadTimers on Teensy 4.0](#5-PWM_Waveform-using-QuadTimers-on-Teensy-40)
* [Debug](#debug)
* [Troubleshooting](#troubleshooting)
* [Issues](#issues)
* [TO DO](#to-do)
* [DONE](#done)
* [Contributions and Thanks](#contributions-and-thanks)
* [Contributing](#contributing)
* [License](#license)
* [Copyright](#copyright)

---
---


### Why do we need this [Teensy_PWM library](https://github.com/khoih-prog/Teensy_PWM)

### Features

This hardware-based PWM library, a wrapper and enhancement around `Teensy PWM` code, enables you to use Hardware-PWM on **Teensy boards, such as Teensy 2.x, Teensy LC, Teensy 3.x, Teensy 4.x, Teensy MicroMod, etc.**, etc. using [Teensyduno core](https://www.pjrc.com/teensy/td_download.html), to create and output PWM. These purely hardware-based PWM channels can generate very high PWM frequencies, depending on CPU clock and acceptable accuracy. The maximum and default resolution is **16-bit** resolution.

This library is using the **same or similar functions** as other FastPWM libraries, as follows, to enable you to **port your PWM code easily between platforms**

 1. [**RP2040_PWM**](https://github.com/khoih-prog/RP2040_PWM)
 2. [**AVR_PWM**](https://github.com/khoih-prog/AVR_PWM)
 3. [**megaAVR_PWM**](https://github.com/khoih-prog/megaAVR_PWM)
 4. [**ESP32_FastPWM**](https://github.com/khoih-prog/ESP32_FastPWM)
 5. [**SAMD_PWM**](https://github.com/khoih-prog/SAMD_PWM)
 6. [**SAMDUE_PWM**](https://github.com/khoih-prog/SAMDUE_PWM)
 7. [**nRF52_PWM**](https://github.com/khoih-prog/nRF52_PWM)
 8. [**Teensy_PWM**](https://github.com/khoih-prog/Teensy_PWM)
 9. [**ATtiny_PWM**](https://github.com/khoih-prog/ATtiny_PWM)
10. [**Dx_PWM**](https://github.com/khoih-prog/Dx_PWM)
11. [**Portenta_H7_PWM**](https://github.com/khoih-prog/Portenta_H7_PWM)
12. [**MBED_RP2040_PWM**](https://github.com/khoih-prog/MBED_RP2040_PWM)
13. [**nRF52_MBED_PWM**](https://github.com/khoih-prog/nRF52_MBED_PWM)
14. [**STM32_PWM**](https://github.com/khoih-prog/STM32_PWM)


---

The most important feature is they're purely hardware-based PWM channels. Therefore, their operations are **not blocked by bad-behaving software functions / tasks**.

This important feature is absolutely necessary for mission-critical tasks. These hardware PWM-channels, still work even if other software functions are blocking. Moreover, they are much more precise (certainly depending on clock frequency accuracy) than other software timers using `millis()` or `micros()`. That's necessary if you need to control external systems (Servo, etc.) requiring better accuracy.

New efficient `setPWM_manual()` function enables waveform creation using PWM.

The [**PWM_Multi**](examples/PWM_Multi) example will demonstrate the usage of multichannel PWM using multiple Hardware-PWM blocks (Timer & Channel). The 4 independent Hardware-PWM channels are used **to control 4 different PWM outputs**, with totally independent frequencies and dutycycles on `Teensy`.

Being hardware-based PWM, their executions are not blocked by bad-behaving functions / tasks, such as connecting to WiFi, Internet or Blynk services.

This non-being-blocked important feature is absolutely necessary for mission-critical tasks.

---

### Why using hardware-based PWM is better

Imagine you have a system with a **mission-critical** function, controlling a robot or doing something much more important. You normally use a software timer to poll, or even place the function in `loop()`. But what if another function is **blocking** the `loop()` or `setup()`.

So your function **might not be executed, and the result would be disastrous.**

You'd prefer to have your function called, no matter what happening with other functions (busy loop, bug, etc.).

The correct choice is to use `hardware-based PWM`.

These hardware-based PWM channels still work even if other software functions are blocking. Moreover, they are much more **precise** (certainly depending on clock frequency accuracy) than other software-based PWMs, using `millis()` or `micros()`.

Functions using normal software-based PWMs, relying on `loop()` and calling `millis()`, won't work if the `loop()` or `setup()` is blocked by certain operation. For example, certain function is blocking while it's connecting to WiFi or some services.

---

### Currently supported Boards

1. **Teensy boards** such as :

  - **Teensy 4.1, Teensy MicroMod, Teensy 4.0**
  - **Teensy 3.6, 3.5, 3.2/3.1, 3.0**
  - **Teensy LC**
  
### To be supported Boards
  
  - **Teensy++ 2.0 and Teensy 2.0**



---
---

## Prerequisites

 1. [`Arduino IDE 1.8.19+` for Arduino](https://github.com/arduino/Arduino). [![GitHub release](https://img.shields.io/github/release/arduino/Arduino.svg)](https://github.com/arduino/Arduino/releases/latest)
 2. [`Teensy core v1.57+`](https://github.com/PaulStoffregen/cores) for Teensy 4.1.  [![GitHub release](https://img.shields.io/github/release/PaulStoffregen/cores.svg)](https://github.com/PaulStoffregen/cores/releases/latest)

---
---

## Installation

### Use Arduino Library Manager

The best and easiest way is to use `Arduino Library Manager`. Search for [**Teensy_PWM**](https://github.com/khoih-prog/Teensy_PWM), then select / install the latest version.
You can also use this link [![arduino-library-badge](https://www.ardu-badge.com/badge/Teensy_PWM.svg?)](https://www.ardu-badge.com/Teensy_PWM) for more detailed instructions.

### Manual Install

Another way to install is to:

1. Navigate to [**Teensy_PWM**](https://github.com/khoih-prog/Teensy_PWM) page.
2. Download the latest release `Teensy_PWM-main.zip`.
3. Extract the zip file to `Teensy_PWM-main` directory 
4. Copy whole `Teensy_PWM-main` folder to Arduino libraries' directory such as `~/Arduino/libraries/`.

### VS Code & PlatformIO

1. Install [VS Code](https://code.visualstudio.com/)
2. Install [PlatformIO](https://platformio.org/platformio-ide)
3. Install [**Teensy_PWM** library](https://registry.platformio.org/libraries/khoih-prog/Teensy_PWM) by using [Library Manager](https://registry.platformio.org/libraries/khoih-prog/Teensy_PWM/installation). Search for **Teensy_PWM** in [Platform.io Author's Libraries](https://platformio.org/lib/search?query=author:%22Khoi%20Hoang%22)
4. Use included [platformio.ini](platformio/platformio.ini) file from examples to ensure that all dependent libraries will installed automatically. Please visit documentation for the other options and examples at [Project Configuration File](https://docs.platformio.org/page/projectconf.html)

---
---

### Packages' Patches

#### 1. For Teensy boards
 
 **To be able to compile and run on Teensy boards**, you have to copy the file [Teensy boards.txt](Packages_Patches/hardware/teensy/avr/boards.txt) into Teensy hardware directory (./arduino-1.8.19/hardware/teensy/avr/boards.txt). 

Supposing the Arduino version is 1.8.19. These files must be copied into the directory:

- `./arduino-1.8.19/hardware/teensy/avr/boards.txt`
- `./arduino-1.8.19/hardware/teensy/avr/cores/teensy/Stream.h`
- `./arduino-1.8.19/hardware/teensy/avr/cores/teensy3/Stream.h`
- `./arduino-1.8.19/hardware/teensy/avr/cores/teensy4/Stream.h`

Whenever a new version is installed, remember to copy this file into the new version directory. For example, new version is x.yy.zz
This file must be copied into the directory:

- `./arduino-x.yy.zz/hardware/teensy/avr/boards.txt`
- `./arduino-x.yy.zz/hardware/teensy/avr/cores/teensy/Stream.h`
- `./arduino-x.yy.zz/hardware/teensy/avr/cores/teensy3/Stream.h`
- `./arduino-x.yy.zz/hardware/teensy/avr/cores/teensy4/Stream.h`

---
---


## Usage

Before using any PWM `Timer` and `channel`, you have to make sure the `Timer` and `channel` has not been used by any other purpose.


#### 1. Create PWM Instance with Pin, Frequency, dutycycle

```cpp
Teensy_PWM* PWM_Instance;

PWM_Instance = new Teensy_PWM(pinToUse, frequency, dutyCycle, channel, PWM_resolution);
```

#### 2. Initialize PWM Instance

```cpp
if (PWM_Instance)
{
  PWM_Instance->setPWM();
}
```

#### 3. Set or change PWM frequency or dutyCycle

To use `float new_dutyCycle`

```cpp
PWM_Instance->setPWM(PWM_Pins, new_frequency, new_dutyCycle);
```

such as 

```cpp
dutyCycle = 10.0f;
  
Serial.print(F("Change PWM DutyCycle to ")); Serial.println(dutyCycle);
PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);
```

---

To use `uint32_t new_dutyCycle` = `(real_dutyCycle * 65536) / 100`


```cpp
PWM_Instance->setPWM_Int(PWM_Pins, new_frequency, new_dutyCycle);
```

such as for `real_dutyCycle = 50%`

```cpp
// 50% dutyCycle = (real_dutyCycle * 65536) / 100
dutyCycle = 32768;

Serial.print(F("Change PWM DutyCycle to (%) "));
Serial.println((float) dutyCycle * 100 / 65536);
PWM_Instance->setPWM_Int(pinToUse, frequency, dutyCycle);
```

for `real_dutyCycle = 50%`

```cpp
// 20% dutyCycle = (real_dutyCycle * 65536) / 100
dutyCycle = 13107;

Serial.print(F("Change PWM DutyCycle to (%) "));
Serial.println((float) dutyCycle * 100 / 65536);
PWM_Instance->setPWM_Int(pinToUse, frequency, dutyCycle);
```

#### 4. Set or change PWM frequency and dutyCycle manually and efficiently in waveform creation

Function prototype

```cpp
bool setPWM_manual(const uint8_t& pin, const uint16_t& DCValue);
```

Need to call only once for each pin


```cpp
PWM_Instance->setPWM(PWM_Pins, frequency, dutyCycle);
```

after that, if just changing `dutyCycle` / `level`, use 

```cpp
PWM_Instance->setPWM_manual(PWM_Pins, new_level);
```

---
---

### Examples: 

 1. [PWM_Basic](examples/PWM_Basic)
 2. [PWM_DynamicDutyCycle](examples/PWM_DynamicDutyCycle)
 3. [PWM_DynamicDutyCycle_Int](examples/PWM_DynamicDutyCycle_Int)
 4. [PWM_DynamicFreq](examples/PWM_DynamicFreq)
 5. [PWM_Multi](examples/PWM_Multi)
 6. [PWM_MultiChannel](examples/PWM_MultiChannel)
 7. [PWM_Waveform](examples/PWM_Waveform)
 8. [PWM_StepperControl](examples/PWM_StepperControl)

 
---
---

### Example [PWM_Multi](examples/PWM_Multi)

https://github.com/khoih-prog/Teensy_PWM/blob/d0c7f2a5c7658e7ae3537431abb5ac62e1357f53/examples/PWM_Multi/PWM_Multi.ino#L11-L113


---
---

### Debug Terminal Output Samples

### 1. PWM_DynamicDutyCycle using FlexTimers on Teensy 4.0

The following is the sample terminal output when running example [PWM_DynamicDutyCycle](examples/PWM_DynamicDutyCycle) using FlexTimers on **Teensy 4.0**, to demonstrate the ability to provide high PWM frequencies and ability to change DutyCycle `on-the-fly`


```cpp
Starting PWM_DynamicDutyCycle using FlexTimers on Teensy 4.0
Teensy_PWM v1.1.1
[PWM] setupPWM: Mapping dutycycle = 0 to newDC = 0 for _resolution = 16
[PWM] setupPWM: Using FlexTimer2 moduleIndex = 1 for PWM pin = 5
=====================================================================================
Change PWM DutyCycle to 90.00
[PWM] setPWM: _dutycycle = 58982 , frequency = 5000.00
[PWM] setPWM_Int: dutycycle = 58982 , frequency = 5000.00
[PWM] setupPWM: Mapping dutycycle = 58982 to newDC = 58982 for _resolution = 16
=====================================================================================
Actual data: pin = 5, PWM DC = 90.00, PWMPeriod = 200.00, PWM Freq (Hz) = 5000.0000
=====================================================================================
Change PWM DutyCycle to 20.00
[PWM] setPWM: _dutycycle = 13107 , frequency = 5000.00
[PWM] setPWM_Int: dutycycle = 13107 , frequency = 5000.00
[PWM] setupPWM: Mapping dutycycle = 13107 to newDC = 13107 for _resolution = 16
=====================================================================================
Actual data: pin = 5, PWM DC = 20.00, PWMPeriod = 200.00, PWM Freq (Hz) = 5000.0000
=====================================================================================
Change PWM DutyCycle to 90.00
[PWM] setPWM: _dutycycle = 58982 , frequency = 5000.00
[PWM] setPWM_Int: dutycycle = 58982 , frequency = 5000.00
[PWM] setupPWM: Mapping dutycycle = 58982 to newDC = 58982 for _resolution = 16
=====================================================================================
Actual data: pin = 5, PWM DC = 90.00, PWMPeriod = 200.00, PWM Freq (Hz) = 5000.0000
=====================================================================================
Change PWM DutyCycle to 20.00
[PWM] setPWM: _dutycycle = 13107 , frequency = 5000.00
[PWM] setPWM_Int: dutycycle = 13107 , frequency = 5000.00
[PWM] setupPWM: Mapping dutycycle = 13107 to newDC = 13107 for _resolution = 16
=====================================================================================
Actual data: pin = 5, PWM DC = 20.00, PWMPeriod = 200.00, PWM Freq (Hz) = 5000.0000
=====================================================================================
```

---

### 2. PWM_Multi using QuadTimers on Teensy 4.0

The following is the sample terminal output when running example [**PWM_Multi**](examples/PWM_Multi) using QuadTimers on **Teensy 4.0**, to demonstrate the ability to provide high PWM frequencies on multiple `PWM-capable` pins

```cpp
Starting PWM_Multi using QuadTimers on Teensy 4.0
Teensy_PWM v1.1.1
[PWM] setupPWM: Mapping dutycycle = 6554 to newDC = 6554 for _resolution = 16
[PWM] setupPWM: Using QuadTimer1 moduleIndex = 0 for PWM pin = 10
[PWM] setPWM_Int: dutycycle = 6554 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 6554 to newDC = 6554 for _resolution = 16
[PWM] setupPWM: Mapping dutycycle = 19661 to newDC = 19661 for _resolution = 16
[PWM] setupPWM: Using QuadTimer1 moduleIndex = 2 for PWM pin = 11
[PWM] setPWM_Int: dutycycle = 19661 , frequency = 3000.00
[PWM] setupPWM: Mapping dutycycle = 19661 to newDC = 19661 for _resolution = 16
[PWM] setupPWM: Mapping dutycycle = 32768 to newDC = 32768 for _resolution = 16
[PWM] setupPWM: Using QuadTimer3 moduleIndex = 2 for PWM pin = 14
[PWM] setPWM_Int: dutycycle = 32768 , frequency = 4000.00
[PWM] setupPWM: Mapping dutycycle = 32768 to newDC = 32768 for _resolution = 16
[PWM] setupPWM: Mapping dutycycle = 58982 to newDC = 58982 for _resolution = 16
[PWM] setupPWM: Using QuadTimer3 moduleIndex = 3 for PWM pin = 15
[PWM] setPWM_Int: dutycycle = 58982 , frequency = 8000.00
[PWM] setupPWM: Mapping dutycycle = 58982 to newDC = 58982 for _resolution = 16
=====================================================================================
Index	Pin	PWM_freq	DutyCycle	Actual Freq
=====================================================================================
0	10	2000.00		10.00		2000.0000
1	11	3000.00		30.00		3000.0000
2	14	4000.00		50.00		4000.0000
3	15	8000.00		90.00		8000.0000
=====================================================================================
Actual data: pin = 10, PWM DC = 10.00, PWMPeriod = 500.00, PWM Freq (Hz) = 2000.0000
=====================================================================================
=====================================================================================
Actual data: pin = 11, PWM DC = 30.00, PWMPeriod = 333.33, PWM Freq (Hz) = 3000.0000
=====================================================================================
=====================================================================================
Actual data: pin = 14, PWM DC = 50.00, PWMPeriod = 250.00, PWM Freq (Hz) = 4000.0000
=====================================================================================
=====================================================================================
Actual data: pin = 15, PWM DC = 90.00, PWMPeriod = 125.00, PWM Freq (Hz) = 8000.0000
=====================================================================================
```

---

### 3. PWM_DynamicFreq using FlexTimers on Teensy 4.0

The following is the sample terminal output when running example [**PWM_DynamicFreq**](examples/PWM_DynamicFreq) using FlexTimers on **Teensy 4.0**, to demonstrate the ability to change dynamically PWM frequencies

```cpp
Starting PWM_DynamicFreq using FlexTimers on Teensy 4.0
Teensy_PWM v1.1.1
[PWM] setupPWM: Mapping dutycycle = 32768 to newDC = 32768 for _resolution = 16
[PWM] setupPWM: Using FlexTimer2 moduleIndex = 1 for PWM pin = 5
=====================================================================================
Change PWM Freq to 20000.00
[PWM] setPWM: _dutycycle = 32768 , frequency = 20000.00
[PWM] setPWM_Int: dutycycle = 32768 , frequency = 20000.00
[PWM] setupPWM: Mapping dutycycle = 32768 to newDC = 32768 for _resolution = 16
=====================================================================================
Actual data: pin = 5, PWM DC = 50.00, PWMPeriod = 50.00, PWM Freq (Hz) = 20000.0000
=====================================================================================
Change PWM Freq to 10000.00
[PWM] setPWM: _dutycycle = 32768 , frequency = 10000.00
[PWM] setPWM_Int: dutycycle = 32768 , frequency = 10000.00
[PWM] setupPWM: Mapping dutycycle = 32768 to newDC = 32768 for _resolution = 16
=====================================================================================
Actual data: pin = 5, PWM DC = 50.00, PWMPeriod = 100.00, PWM Freq (Hz) = 10000.0000
=====================================================================================
```


---


### 4. PWM_Waveform using FlexTimers on Teensy 4.0

The following is the sample terminal output when running example [**PWM_Waveform**](examples/PWM_Waveform) using FlexTimers on **Teensy 4.0**, to demonstrate how to use the `setPWM_manual()` function in wafeform creation


```cpp
Starting PWM_Waveform using FlexTimers on Teensy 4.0
Teensy_PWM v1.1.1
[PWM] setupPWM: Mapping dutycycle = 0 to newDC = 0 for _resolution = 16
[PWM] setupPWM: Using FlexTimer2 moduleIndex = 1 for PWM pin = 5
[PWM] setPWM: _dutycycle = 0 , frequency = 2000.00
[PWM] setPWM_Int: dutycycle = 0 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 0 to newDC = 0 for _resolution = 16
============================================================================================
Actual data: pin = 5, PWM DutyCycle = 0.00, PWMPeriod = 500.00, PWM Freq (Hz) = 2000.0000
============================================================================================
[PWM] setPWM_manual: _dutycycle = 0 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 0 to newDC = 0 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 3276 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 3276 to newDC = 3276 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 6553 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 6553 to newDC = 6553 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 9830 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 9830 to newDC = 9830 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 13107 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 13107 to newDC = 13107 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 16383 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 16383 to newDC = 16383 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 19660 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 19660 to newDC = 19660 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 22937 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 22937 to newDC = 22937 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 26214 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 26214 to newDC = 26214 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 29490 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 29490 to newDC = 29490 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 32767 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 32767 to newDC = 32767 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 36044 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 36044 to newDC = 36044 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 39321 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 39321 to newDC = 39321 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 42597 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 42597 to newDC = 42597 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 45874 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 45874 to newDC = 45874 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 49151 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 49151 to newDC = 49151 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 52428 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 52428 to newDC = 52428 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 55704 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 55704 to newDC = 55704 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 58981 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 58981 to newDC = 58981 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 62258 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 62258 to newDC = 62258 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 65535 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 65535 to newDC = 65535 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 62258 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 62258 to newDC = 62258 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 58981 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 58981 to newDC = 58981 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 55704 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 55704 to newDC = 55704 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 52428 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 52428 to newDC = 52428 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 49151 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 49151 to newDC = 49151 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 45874 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 45874 to newDC = 45874 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 42597 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 42597 to newDC = 42597 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 39321 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 39321 to newDC = 39321 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 36044 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 36044 to newDC = 36044 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 32767 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 32767 to newDC = 32767 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 29490 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 29490 to newDC = 29490 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 26214 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 26214 to newDC = 26214 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 22937 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 22937 to newDC = 22937 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 19660 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 19660 to newDC = 19660 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 16383 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 16383 to newDC = 16383 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 13107 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 13107 to newDC = 13107 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 9830 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 9830 to newDC = 9830 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 6553 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 6553 to newDC = 6553 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 3276 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 3276 to newDC = 3276 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 0 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 0 to newDC = 0 for _resolution = 16
```

---

### 5. PWM_Waveform using QuadTimers on Teensy 4.0

The following is the sample terminal output when running example [**PWM_Waveform**](examples/PWM_Waveform) using QuadTimers on **Teensy 4.0**, to demonstrate how to use the `setPWM_manual()` function in wafeform creation


```cpp
Starting PWM_Waveform using QuadTimers on Teensy 4.0
Teensy_PWM v1.1.1
[PWM] setupPWM: Mapping dutycycle = 0 to newDC = 0 for _resolution = 16
[PWM] setupPWM: Using QuadTimer3 moduleIndex = 3 for PWM pin = 15
[PWM] setPWM: _dutycycle = 0 , frequency = 2000.00
[PWM] setPWM_Int: dutycycle = 0 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 0 to newDC = 0 for _resolution = 16
============================================================================================
Actual data: pin = 15, PWM DutyCycle = 0.00, PWMPeriod = 500.00, PWM Freq (Hz) = 2000.0000
============================================================================================
[PWM] setPWM_manual: _dutycycle = 0 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 0 to newDC = 0 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 3276 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 3276 to newDC = 3276 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 6553 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 6553 to newDC = 6553 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 9830 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 9830 to newDC = 9830 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 13107 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 13107 to newDC = 13107 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 16383 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 16383 to newDC = 16383 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 19660 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 19660 to newDC = 19660 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 22937 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 22937 to newDC = 22937 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 26214 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 26214 to newDC = 26214 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 29490 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 29490 to newDC = 29490 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 32767 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 32767 to newDC = 32767 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 36044 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 36044 to newDC = 36044 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 39321 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 39321 to newDC = 39321 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 42597 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 42597 to newDC = 42597 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 45874 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 45874 to newDC = 45874 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 49151 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 49151 to newDC = 49151 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 52428 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 52428 to newDC = 52428 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 55704 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 55704 to newDC = 55704 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 58981 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 58981 to newDC = 58981 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 62258 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 62258 to newDC = 62258 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 65535 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 65535 to newDC = 65535 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 62258 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 62258 to newDC = 62258 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 58981 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 58981 to newDC = 58981 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 55704 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 55704 to newDC = 55704 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 52428 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 52428 to newDC = 52428 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 49151 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 49151 to newDC = 49151 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 45874 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 45874 to newDC = 45874 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 42597 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 42597 to newDC = 42597 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 39321 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 39321 to newDC = 39321 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 36044 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 36044 to newDC = 36044 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 32767 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 32767 to newDC = 32767 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 29490 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 29490 to newDC = 29490 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 26214 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 26214 to newDC = 26214 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 22937 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 22937 to newDC = 22937 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 19660 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 19660 to newDC = 19660 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 16383 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 16383 to newDC = 16383 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 13107 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 13107 to newDC = 13107 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 9830 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 9830 to newDC = 9830 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 6553 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 6553 to newDC = 6553 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 3276 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 3276 to newDC = 3276 for _resolution = 16
[PWM] setPWM_manual: _dutycycle = 0 , frequency = 2000.00
[PWM] setupPWM: Mapping dutycycle = 0 to newDC = 0 for _resolution = 16
```

---
---

### Debug

Debug is enabled by default on Serial.

You can also change the debugging level `_PWM_LOGLEVEL_` from 0 to 4

```cpp
// Don't define _PWM_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define _PWM_LOGLEVEL_     0
```

---

### Troubleshooting

If you get compilation errors, more often than not, you may need to install a newer version of the core for Arduino boards.

Sometimes, the library will only work if you update the board core to the latest version because I am using newly added functions.


---
---

### Issues

Submit issues to: [Teensy_PWM issues](https://github.com/khoih-prog/Teensy_PWM/issues)

---
---

## TO DO

1. Search for bug and improvement.
2. Support to **Teensy 2.x**

---

## DONE

 1. Basic hardware PWM-channels for **Teensy 4.x boards, such as Teensy 4.0, Teensy 4.1, Teensy MicroMod, etc.**, using [Teensyduno core](https://www.pjrc.com/teensy/td_download.html).
 2. Add support to **Teensy 3.x and Teensy LC**
 3. Add example [PWM_StepperControl](https://github.com/khoih-prog/Teensy_PWM/examples/PWM_StepperControl) to demo how to control Stepper Motor using PWM
 
---
---

### Contributions and Thanks

Many thanks for everyone for bug reporting, new feature suggesting, testing and contributing to the development of this library.

1. Thanks to [Paul van Dinther](https://github.com/dinther) for proposing new way to use PWM to drive Stepper-Motor in [Using PWM to step a stepper driver #16](https://github.com/khoih-prog/RP2040_PWM/issues/16), leading to v2.0.3


<table>
  <tr>
    <td align="center"><a href="https://github.com/dinther"><img src="https://github.com/dinther.png" width="100px;" alt="dinther"/><br /><sub><b>Paul van Dinther</b></sub></a><br /></td>
  </tr>
</table>


---

## Contributing

If you want to contribute to this project:
- Report bugs and errors
- Ask for enhancements
- Create issues and pull requests
- Tell other people about this library

---

### License

- The library is licensed under [MIT](https://github.com/khoih-prog/Teensy_PWM/blob/main/LICENSE)

---

## Copyright

Copyright (c) 2022- Khoi Hoang


