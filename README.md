# FGDOS_Arduino
Arduino code to interact with the FGDOS. Test and CERN programs included as well.
Notes:
- Older programs consist of several files inside the same specific program folder. This was done to split functions and definitions for the general program loop, increasing readability. Newer programs, however, all make use of a separate Arduino library (that includes all the functions and definitions, again, to increase readability). Library can be provided upon request.

### CERN
Code provided by CERN. Includes an adapted version that I changed as to interact with my specific setup.

### FGDOS

### I2C

### PC_FGDOS_02F
Programs to interact with the FGD-02F sensor. V02 is just an update from V01, command includes a function that reads one letter command inputs from the user.

### PC_FGDOS_03F
Programs to interact with the FGD-03F. CR is the command and read function, you should be able to to everything with this. It reads sensor data and sends it to the PC. You can also command the Arduino in real time. The passive programs were an attempt to automate passive mode usage of the sensor, but timing this proved to be difficult. The command program is an older version of the CR program.

### PC_FGDOS_03F_SINGLE
The FGD-03F prototype only contains one sensor. To interact with the sensor, you can just use the programs abovve and slightly adapt them. However, as to not meddle too much with the settings in the programs above, separate programs were created for this purpose. 

### FGDOS_freq_registers
Because the target and threshold frequency registers of the sensor only store a limited amount of bits, conversion to actual frequency values of the registers is not always straightforward. It also depends on the clock and window pulses settings. The excel file can be used for some basic conversions.

