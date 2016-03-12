CV-MonoSLAM
===========

Introduction
------------

This project proposes a new approach of monocular ceiling vision based simultaneous localization and mapping (SLAM) by utilizing an improved Square Root Unscented Kalman Filter (SRUKF). With a monocular camera mounted on the top of a mobile robot and looking upward to the ceiling, the robot only needs to process salient features, which greatly reduce the computational complexity and have a high accuracy. SRUKF is used instead of the standard Extended Kalman Filter (EKF) to improve the linearization problem in both motion and perception models. To address the numerical instability problems in the standard SRUKF, several optimization methods are utilized in the project. 

![image](https://github.com/mejliu/CV-MonoSLAM/raw/master/MonoSLAM/Images/TurtleBot.PNG)

Requirements
---------------

This project was developed under Microsoft Visual Studio 2010, and depends on the following software:
* OpenCV v2.4.3.
* GSL GnuWin32 v1.8.
* OpenGL.


Note: The ceiling vision test images is too large to upload.

***For algorithm, please follow this [link][link]***

[link]:http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=6739701

[![Bitdeli Badge](https://d2weczhvl823v0.cloudfront.net/mejliu/cv-monoslam/trend.png)](https://bitdeli.com/free "Bitdeli Badge")
