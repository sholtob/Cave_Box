# Cave_Box
Firmware and schematic for the Cave Box humidity controller system.  

For more information about the Cave Box please visit https://sholtosworkshop.com/.


You can see what version of the Cave Box software is installed on your device by going to 
"Settings->Advanced->Version".
Version 1.02 allows the spare pins on the left of the board to be used to control solid state relays. 
Starting at the bottom pin (labelled J1 on the PCB) pinout is (GND, 5V, 3V3, LED, Humidifier, Fan, unassigned, unassigned).
You can solder wires to these contacts and use them to switch solid state relays controlling much bigger fans, a more powerful humidifier system ect.

Version 1.03 included support for the AHT20 humidity sensor (these are black and have 4 wires). It is backwards compatible with the DHT22 sensor (white and has 3 wires) used in the earlier Cave Boxes. Simply uncomment the relevent "#define USE_AHT20_SENSOR/USE_DHT22_SENSOR" line and comment out the one you don't want. 

Version 1.04 reduces the Factory Reset Max RH value to 96% to try to protect the humidity sensors from condensation. It also allows the fan to run if the tach signal is not present. Previously if the tach signal couldn't be read for whatever reason the fan couldn't be used even if it was still fully functional in all other respects. Now the lack of tach signal just produces a warning message on startup. 
