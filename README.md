# Watchman
Code for AIY Vision face tracking eye mechanism

### Arduino
  Arduino .ino files run on the Arduino Uno with and Adafruit 16 channel servo shield (https://www.adafruit.com/product/1411)

### Python/Raspberry Pi
  Eye_Mechanism_Serial_Adv runs on the Raspberry pi with AIY Vision hat. This does the face-tracking heavy lifting.
  
  Camera_Preview_Image_Overlay is a live camera feed with a crosshair - used with laser eyes to calibrate servo values in Serial_Servo_Control_Adv
  
  Crosshair_Black.png is used by Camera_Preview_Image_Overlay to create a crosshairs over the image
