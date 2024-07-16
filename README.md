# Cave_Box
Firmware and schematic for the Cave Box humidity controller system.  

For more information about the Cave Box please visit https://sholtosworkshop.com/.


You can see what version of the Cave Box software is installed on your device by going to 
"Settings->Advanced->Version".
Version 1.02 allows the spare pins on the left of the board to be used to control solid state relays. 
Starting at the bottom pin (labelled J1 on the PCB) pinout is (GND, 5V, 3V3, LED, Humidifier, Fan, unassigned, unassigned).
You can solder wires to these contacts and use them to switch solid state relays controlling much bigger fans, a more powerful humidifier system ect.

Version 1.03 included support for the AHT20 humidity sensor (these are black and have 4 wires). It is backwards compatible with the DHT22 sensor (white and has 3 wires) used in the earlier Cave Boxes. Simply uncomment the relevent "#define USE_AHT20_SENSOR/USE_DHT22_SENSOR" line and comment out the one you don't want. 

If you want to build this code please use version 2.0.17 of the esp32 board package.
