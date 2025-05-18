# RoboArm using the MSP430FR2355 wwith the PCA9685

## 🖼️ Preview (GIF)
![Servo Demo](assets/roboGif.gif)

This project demonstrates how to use a **MSP430FR2355** microcontroller to control servos via a **PCA9685 I2C servo driver**. The servo positions are dynamically set using a slave arm with **four potentiometers** connected to the ADC, and a **push button** toggles an additional servo for grip.

## 🧩 Components Used
- MSP430FR2355 LaunchPad
- PCA9685 Servo Driver (I2C, 16-channel, Adafruit)
- 4x Potentiometers (connected to P1.1, P1.4, P1.5, P1.6)
- 4x Servos (connected to PCA9685 channels 0, 3, 4, 8, 11)
- 1x Push Button (on P1.7, with pull-up resistor)

## ⚙️ Features
- I2C communication with PCA9685
- PWM frequency set to 50 Hz
- ADC reads 4 analog inputs from potentiometers
- Push button triggers additional servo movement and LED toggle
- Low power operation using `LPM0` during ADC and I2C waits

## 🖼️ System Overview
[ Potentiometers ] --> [ MSP430 ADC ] --> [ PCA9685 PWM Out ] --> [ Servo ]
[   Button Input ] --> [    GPIO     ] --> [ LED Toggle & Servo Pulse ]

## 🧠 Code Highlights
- `init_I2C(addr)` configures I2C master mode
- `init_ADC()` sets up the 12-bit ADC for 4 input channels
- PWM frequency configured using register 0xFE = `121` → 50 Hz
- `set_servo_position(channel, position)` sends position to PCA9685
- Interrupt-driven:
  - `EUSCI_B0_I2C_ISR` handles I2C transmission
  - `ADC_ISR` saves ADC results per input channel
  - `Port_1` responds to button press

## 🛠️ Build & Flash
Use **TI Code Composer Studio (CCS)** or `msp430-gcc` to build and flash the code to your MSP430FR2355 device.

## 📊 Mapping Potentiometer to Servo Position

```C
uint16_t position = ((4095 - ADC_Result_X) \* 700) / 4095 + 60;
```

This scales the 12-bit ADC range to approx. 60–760, matching the pulse width range for most hobby servos via PCA9685 (12-bit resolution @ 50Hz).

## 🧪 Runtime Behavior
- Turning the potentiometers (moving the slave arm) moves the assigned servos in real-time.
- Pressing the button opens the gripper.
- Releasing closes it
