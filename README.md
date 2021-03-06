# DIY__wi-fi-alarm_IOT

## Basic Info

This alarm system consists of several Wi-Fi (Wireless-Fidelity) alarm devices base on TI's CC3200 microcontroller, a wireless router, an embedded cloud server platform and smartphone client. The Wi-Fi alarm device utilizes the tri-axial accelerometer to monitor the acceleration of the object by means of a certain algorithm, which can determine the motion of the object. The alarm message produced by the device is transmitted to the cloud server through the wireless router and finally is forwarded to user's mobile phone by the cloud server. In addition, the alarm system makes full use of the low-power and embedded Wi-Fi SoC architecture of CC3200, whilch can switch normally among the normal state, low-power state and sleep state. Therefore, the alarm device has the advantage of lower power consumption.   



嵌入式低功耗 Wi-Fi 物联网报警系统主要由多个基于CC3200微控制器的Wi-Fi报警装置、无线路由器、嵌入式云服务器、智能手机客户端组成。该Wi-Fi报警装置上的三轴加速度传感器，通过相关算法实时对物体的加速度信息进行监控，判断出被测物体的运动状态。当物体的加速度发生变化时，该报警装置将相关报警信息通过无线路由器转发至云服务器，最终由云服务器将报警信息发送到已绑定的用户手机来进行相关的报警操作。另外，该报警系统充分利用了CC3200的低功耗、嵌入式Wi-Fi SoC的特点，能在正常状态、低功耗状态和休眠状态之间进行正常切换。因此，报警装置具有较低的功耗。



## Modules

This project including 3 modules:

- firmware:  Wi-Fi alarm based on CC3200 uC
- Cloud APP：based on php
- Android APP: based on Java
