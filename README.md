# STM32 iBUS Motor Driver Firmware

Differential-drive motor control firmware for a small UGV, built on an STM32 Nucleo-F767ZI. A FlySky i6 transmitter and FS-iA6B receiver send drive commands over the iBUS protocol; the STM32 mixes them into left/right wheel commands and drives two BLDC motor controllers.

## Status

| Subsystem | Status |
|---|---|
| iBUS RC reception (USART6) | Working and verified |
| Debug console over USB (USART3) | Working |
| Motor control firmware | Working, verified straight-line driving and turning |
| Arm/disarm button | Implemented, not yet field tested |

The vehicle has two driven wheels (differential drive) and two free rear caster wheels.

## Hardware

- MCU board: STM32 Nucleo-F767ZI
- Motor controllers: two ZS-X11H BLDC controllers (6-60 VDC, Hall-sensor commutation), one per driven wheel
- RC transmitter: FlySky i6
- RC receiver: FlySky FS-iA6B, connected via its iBUS (servo) port

<img src="images/hardware-photo.jpg" width="720" alt="Electronics enclosure mounted on the robot, showing the battery, STM32 Nucleo board, and motor controllers">

Battery, STM32 Nucleo board, and motor controllers inside the robot's electronics enclosure.

## How it works

1. The receiver streams a 32-byte iBUS frame roughly every 7 ms over USART6. The frame is captured with DMA and an idle-line interrupt, so the CPU is not interrupted on every byte.
2. The right stick's vertical axis is used as speed and the horizontal axis as steering. Both are spring-centered, so releasing the stick brings the robot to a stop.
3. The firmware mixes these into a left and right wheel command using a simple differential-drive formula (left = speed + turn, right = speed - turn), applies a deadband, and clamps the result.
4. Each wheel command is turned into a PWM duty cycle (10 kHz) plus a direction signal and a run/stop signal, sent to the corresponding ZS-X11H controller.
5. If no valid RC frame arrives for 100 ms, or the vehicle is disarmed, both motors are stopped immediately.
6. A momentary push button toggles the robot between armed and disarmed. The robot powers up disarmed, so it will not move until the button is pressed once.

<img src="images/block-diagram.svg" width="720" alt="Block diagram: RC transmitter to receiver to STM32, arm button into STM32, STM32 to two motor controllers to two wheel motors">

Signal and control flow from the RC transmitter to the wheel motors.

## Wiring summary

All motor control signals are on GPIO port E.

| Signal | STM32 pin | Notes |
|---|---|---|
| Left motor speed (PWM) | PE9 (TIM1 channel 1) | to controller's P pin |
| Left motor direction | PE10 | active low |
| Left motor stop | PE12 | active low, high = run |
| Right motor speed (PWM) | PE11 (TIM1 channel 2) | to controller's P pin |
| Right motor direction | PE13 | active low |
| Right motor stop | PE14 | active low, high = run |
| Arm/disarm button | PE15 | input with internal pull-up |
| RC receiver iBUS signal | PC7 (USART6 RX) | 115200 baud, 8N1, not inverted |

Both motor controllers share a common ground with the STM32. Full pinout details, jumper settings, and safety notes are in the project's design log.

## Repository layout

```
Motor_driver/
  Core/Inc/          headers (main, motor, ibus, LCD driver)
  Core/Src/          application source, including motor.c and ibus.c
  Core/Startup/      startup assembly file
  Drivers/           STM32 HAL and CMSIS files
  Motor_driver.ioc    STM32CubeMX project configuration
  RC_DRIVE_NOTES.md   detailed design and development log
images/
  block-diagram.svg   signal and control flow diagram
  hardware-photo.jpg   photo of the electronics enclosure
README.md
```

## Building and flashing

This is an STM32CubeIDE project.

1. Open the project folder in STM32CubeIDE.
2. Build the project (a first build inside the IDE is required so new source files are picked up by the build system).
3. Flash the board using STM32CubeIDE, or with the generated makefile in the `Debug` folder using `st-flash` or `openocd`.
4. Connect to the debug console at 115200 baud over the board's USB port to see status output.

## Safety notes

- Test with the wheels off the ground and the motor supply current-limited before running the vehicle on the ground.
- Verify that turning the transmitter off causes both motors to stop and the debug console to report a failsafe state.
- Confirm the arm/disarm button behaves as expected before relying on it.

## Further reading

See `RC_DRIVE_NOTES.md` in this repository for the full design log, including the iBUS frame format, motor controller pinout, PWM configuration, and known open issues.
