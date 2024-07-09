This is my modified fork.
The original files are here: https://github.com/robotzero1/esp32cam-access-control
# ESP32-CAM Access Control System
Using an ESP32-CAM to unlock a door when a face is recognised.  A simple access control system based on face recognition with the ESP32-CAM and the Arduino IDE.
WEB interface URL: http://esp32-arduino.lan/

![Interface](https://robotzero.one/wp-content/uploads/bfi_thumb/featured-master-1-6qb32y1978bm9yf6ne9nypfey83yf3pvisk7rozvfi8.jpg)  

Note: If you are using release 1.0.5 of the ESP32 Arduino Hardware Libraries you can now run the entire project with just these files

FaceDoorEntryESP32Cam.ino  
camera_index.h  
camera_pins.h  
partitions.csv  

There is no need to create new partitions in the Arduino IDE  

Note: Releases 2.x.x of the ESP32 Hardware Libraries don't support face recognition on the ESP32 modules. The algorithm has been 'improved' and now runs too slowly on these processors.

Full details on the blog: https://robotzero.one/access-control-with-face-recognition/

Step 1
Put files in a folder:
![Interface](https://raw.githubusercontent.com/US4LKW/ESP32CAM-Face-ID-door-lock/master/Put_files.jpg)

Step 2
Open the file FaceDoorEntryESP32Cam.ino in Arduino IDE and select the board version for ESP32:
![Interface](https://raw.githubusercontent.com/US4LKW/ESP32CAM-Face-ID-door-lock/master/Install_board.jpg)

Step 3
Select partition:
![Interface](https://raw.githubusercontent.com/US4LKW/ESP32CAM-Face-ID-door-lock/master/Select_partition.jpg)

Enjoy!
