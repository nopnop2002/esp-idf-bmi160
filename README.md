# esp-idf-bmi160
A demo showing the pose of the bmi160 in 3D using esp-idf.

BMI160 is a 6DoF IMU.   
You can use the Kalman filter or the Madgwick filter to estimate the Euler angle.   
Euler angles are roll, pitch and yaw.   
![a-Pitch-yaw-and-roll-angles-of-an-aircraft-with-body-orientation-O-u-v-original](https://user-images.githubusercontent.com/6020549/226072914-a7f923fc-eb6e-4d19-b2ff-8c9f2749ee6f.jpg)

View Euler angle with [this](https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation) 3D viewer.   

I used [this](https://github.com/boschsensortec/BMI160_driver) library.   

# Installation overview

- Install a 3D viewer.   

- Get Euler angles from bmi180 and display them in 3D.

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

# Install a 3D viewer   
I used pyteapot.py.   
It works as a UDP display server.   
```
+-------------+     +-------------+     +-------------+
|     IMU     | i2c |    ESP32    | UDP | pyteapot.py |
|             |---->|             |---->|             |
|             |     |             |     |             |
+-------------+     +-------------+     +-------------+
```

This is a great application.   

```
$ sudo apt install python3-pip python3-setuptools
$ python3 -m pip install -U pip
$ python3 -m pip install pygame
$ python3 -m pip install PyOpenGL PyOpenGL_accelerate
$ git clone https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation
$ cd PyTeapot-Quaternion-Euler-cube-rotation
$ python3 pyteapot.py
```

If this screen is displayed, the installation was successful.
![pyteapot_2023-03-10_17-25-56](https://user-images.githubusercontent.com/6020549/224452171-3c65c911-b1c6-4e92-b1f8-ea4ee88ecbee.png)


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


The posture of your sensor is displayed.   
![bmi180_2023-03-17_06-08-57](https://user-images.githubusercontent.com/6020549/226072858-b5e52dc5-db87-4613-8c05-4008a4bb8170.png)


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

The posture of your sensor is displayed.   
![bmi180_2023-03-17_06-08-57](https://user-images.githubusercontent.com/6020549/226072858-b5e52dc5-db87-4613-8c05-4008a4bb8170.png)

