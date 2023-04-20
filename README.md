# esp-idf-bmi160
A demo showing the pose of the bmi160 6DoF IMU sensor in 3D using esp-idf.

You can use the Kalman filter or the Madgwick filter to estimate the Euler angle.   
Euler angles are roll, pitch and yaw.   
It's very intuitive and easy to understand.   
However, since BMI160 is a 6DoF IMU, YAW estimation is not possible.   
![a-Pitch-yaw-and-roll-angles-of-an-aircraft-with-body-orientation-O-u-v-original](https://user-images.githubusercontent.com/6020549/226072914-a7f923fc-eb6e-4d19-b2ff-8c9f2749ee6f.jpg)

I used [this](https://github.com/boschsensortec/BMI160_driver) library.   

# Software requiment
ESP-IDF V4.4/V5.0.   
ESP-IDF V5 is required when using ESP32-C2.   

# Hardware requirements
BMI160 Accelerometer Gyroscope module 6 Dof inertial Measurement Sensors.   

# Wireing
|BMI160||ESP32|ESP32-S2/S3|ESP32-C2/C3||
|:-:|:-:|:-:|:-:|:-:|:-:|
|VIN|--|N/C|N/C|N/C||
|3V3|--|3.3V|3.3V|3.3V||
|GND|--|GND|GND|GND||
|SCL|--|GPIO22|GPIO12|GPIO5|(*1)|
|SDA|--|GPIO21|GPIO11|GPIO4|(*1)|
|CS|--|3.3V|3.3V|3.3V|Use i2c|
|SAO|--|GND/3.3V|GND/3.3V|GND/3.3V|(*2)|

(*1)You can change it to any pin using menuconfig.   

(*2)Choosing an i2c address.   
GND:i2c address is 0x68.   
3.3V:i2c address is 0x69.   


# Get Euler angles from bmi160 using Kalman filter
I used [this](https://github.com/TKJElectronics/KalmanFilter).
```
git clone https://github.com/nopnop2002/esp-idf-bmi160
cd esp-idf-bmi160/Kalman
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3}
idf.py menuconfig
idf.py flash
```

### Configuration
![config-top](https://user-images.githubusercontent.com/6020549/226072705-e26bfc3e-1a70-4df2-98c9-a603daf58761.jpg)
![config-app](https://user-images.githubusercontent.com/6020549/226072711-ce59da34-bf0e-4bc2-8305-851f5d418097.jpg)


# Get Euler angles from bmi160 using Madgwick filter
I used [this](https://github.com/arduino-libraries/MadgwickAHRS).
```
git clone https://github.com/nopnop2002/esp-idf-bmi160
cd esp-idf-bmi160/Madgwick
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3}
idf.py menuconfig
idf.py flash
```

### Configuration
![config-top](https://user-images.githubusercontent.com/6020549/226072705-e26bfc3e-1a70-4df2-98c9-a603daf58761.jpg)
![config-app](https://user-images.githubusercontent.com/6020549/226072711-ce59da34-bf0e-4bc2-8305-851f5d418097.jpg)

# View Euler angles with built-in web server   
ESP32 acts as a web server.   
I used [this](https://github.com/Molorius/esp32-websocket) component.   
This component can communicate directly with the browser.   
It's a great job.   
Enter the following in the address bar of your web browser.   
```
http:://{IP of ESP32}/
or
http://esp32.local/
```

![bmi160-euler](https://user-images.githubusercontent.com/6020549/232381988-f4003a78-2145-493c-829e-9d0952117ea6.JPG)

WEB pages are stored in the html folder.   
I used [this](https://canvas-gauges.com/) for gauge display.   
I used [this](https://threejs.org/) for 3D display.   
You can change it as you like.   


# View Euler angles using PyTeapot   
You can view Euler angles using [this](https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation) tool.   
It works as a UDP display server.   
This is a great application.   

```
+-------------+     +-------------+     +-------------+
|     IMU     | i2c |    ESP32    | UDP | pyteapot.py |
|             |---->|             |---->|             |
|             |     |             |     |             |
+-------------+     +-------------+     +-------------+
```

### Installation
```
$ sudo apt install python3-pip python3-setuptools
$ python3 -m pip install -U pip
$ python3 -m pip install pygame
$ python3 -m pip install PyOpenGL PyOpenGL_accelerate
$ git clone https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation
$ cd PyTeapot-Quaternion-Euler-cube-rotation
$ python3 pyteapot.py
```
The posture of your sensor is displayed.   
![bmi180_2023-03-17_06-08-57](https://user-images.githubusercontent.com/6020549/226072858-b5e52dc5-db87-4613-8c05-4008a4bb8170.png)

