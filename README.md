# CartPositionControl
Control a cart position by driving a dc motor and read its position with a two channel optical encoder

-The position is read by interrupts on pins 2 and 3. (0 to 28000 encoder steps aprox)
-With calculated position a PID compensator computes how much duty cycle should give to motor driver (Robotdyn L293 h-bridge based Motorshield) as a number between -255 and 255 to achieve desired position.
-The driveMotor(mv) function decides how to drive the shield based on sign of the PID output.

Connection Scheme

EncoderChannelA ----> D2    D11--->MotorShield DutyCycle
Encode ChannelB ----> D3    D13--->MotorShield CC or CCW

