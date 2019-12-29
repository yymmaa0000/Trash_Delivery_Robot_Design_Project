# Trash_Delivery_Robot_Design_Project
This is a group project where we designed a autonomous trash delivery robot to help households deliver their trash from home to curb.
The duration of the project was from Jan - May 2019, and was just uploaded to my Github account recently.

This robot uses Arduino Mega2560 as its microcontroller and can memorize the path from home to curb to automatically deliver trash. 
The project has integrated encoders, compass sensor, and Bluetooth module to achieve reliable path memorization with tracking error less than 2%.
The calibration.ino and odometry.ino files show the detailed implementation.

The motion of the robot is implemented by motion.ino file. We used skid-steering as the turning mechanism, and 
we also implemented robust PI controller to maintain constant motor speed for smooth turning and terrain adaptability.

Final poster can be found [here](https://github.com/yymmaa0000/Trash_Delivery_Robot_Design_Project/blob/master/Documentation/Final%20Poster.pdf).

Final product:

![alt text](https://github.com/yymmaa0000/Trash_Delivery_Robot_Design_Project/blob/master/Documentation/Final%20product.jpg)

CAD rendering:

![alt text](https://github.com/yymmaa0000/Trash_Delivery_Robot_Design_Project/blob/master/Documentation/CAD.jpg)


