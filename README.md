# teensyIMU
IMU using Teensyduino2.0 and Adafruit 9DOF breakout sensor.

# Purpose
My goal is to make this a defacto repository and tutorial for building inexpensive DIY IMU head tracking solution for use in flight simulator and racing games, or any other such solution. Currently it supports Adafruit's NXP FXOS/FXAS 9DoF breakout board, but plan to add LSM6DS33, LSM6DS3U, LSM6DSOX, LSM9DS1 or LSM9DS0 functionality later.

# Hardware
1x Teensyduino 2.0 (or newer, or any similar arduino that can emulate joystick device) (https://www.pjrc.com/store/teensy.html )
1x Adafruit 9DoF sensor breakout board ( https://www.adafruit.com/product/3463 )
1x Interpreter software, such as OpenTrack or FaceTrackNOIR that can translate joystick inputs into as a head tracker/trackIR output.

# Current Progress
At this time I am running into an issue with the yaw data output. I am not sure whether it is the way that I am updating the values or improper calibration of the magnetometer, but the output yaw values are exhibiting excessive drift. If possible, I am looking for contributors that can look over the code and offer improvements.

## Other Notes
  - need to refactor readme
  - provide a full hardware set up guide
