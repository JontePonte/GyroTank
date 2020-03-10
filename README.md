# GyroTank
An RC-tank stabilized with a MPU6050 gyro

## Version 1.0
Version 1.0 done before uploaded to github
### Major Features
- The tank reads rc-input using an interupt sequence
- The tank reads gyro and accelerometer signals from the GY-521
- The pwm-signals to the esc:s are working with manageable gitter
- The esc-signals are controlled by combining throttle and PID-output from z-axis (turning) and (y-axis) anti-wheele

### Minor Features
- Anti-go-crazy-while-upside-down feature
- Deadband on anti-wheele-system
- Shutdown while trottle (channel_1) is down

## Roadmap
### Version 1.1
-Set mode switch on a rc-switch
- Create a mode that shuts down gyro stabilization
- Add FPV with a rc controlled camera
- Create new main body for the physical tank
- Upload .stl or fusion360-files to github

### Version 1.2
- Create a mode that balance the tank standning up
- Create new tracks with suspension
- Make the tank waterproof
