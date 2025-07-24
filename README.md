# Ball_Beam_PID_Control

## Description:

#### The Ball and Beam project is a simple way to learning basic PID control. In the system, the inputs are from the VL53L1X time of flight sensor that gives the distance of the ball relative to the sensor. This is put through a PID control function that will account for the proportional change, the velocity of the ball, and the past inputs to create an output. This output (a number) is then used to control the actuator (the servo motor) to tilt the beam accordingly in order to keep the ball in the middle.

## Hardware:

#### - STM32 Nucleo F446RE
#### - Miuzei MG996R Servo Motor 
#### - VL53L1X 4m Ranging Sensor (CJMCU-531 Breakout Board)
#### - 3D Print Model from https://grabcad.com/library/ball-on-beam-1

## Software:

#### - Using STM32 CubeIDE
#### - VL53L1X ULD API (Ultra Lite Driver Application Programming Interface) from https://www.st.com/en/embedded-software/stsw-img009.html
#### - Servo Control Function (Manually Wrote)
#### - PID Control Function (Manually Wrote)
#### - Main Code Loop (Manually Wrote)

