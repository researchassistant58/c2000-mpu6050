# c2000-mpu6050
Raw data exchange between c2000 Piccolo Launchpad and MPU6050 
with using I2C and print out to serial monitor (Putty, etc.)

Some comments are missing.

The code is verified by CCS v6 and C2000 piccolo launchpad (TMS320F28027) and Putty.

Codes are modified from example projects and shared codes in e2e forum provided by TI.

In c2000 launchpad S1 Switch is ON-ON-ON mode

S4 Switch is ON mode due to serial connection between device and host PC.

If you want to program flash memory of c2000 launchpad then go and find example flash program in Controlsuite software
C:\ti\controlSUITE\device_support\f2802x\v230\f2802x_examples_drivers\flash_f2802x  
change the original source code to my code and build-run.

Best wishes...



