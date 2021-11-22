# pod-mcu-code

This is for the IMU devices.

* Hardware Overview

Hardware used: LilyGo TTGO Watch 2019 with an added accelerometer.  

![LilyGo TTGO Watch](https://imgaz.staticbg.com/thumb/large/oaupload/banggood/images/44/92/e8ef07ff-9060-4fc7-92cc-63c0b4886854.jpg)


*[You can purchase this hardware here](https://usa.banggood.com/LILYGO-TTGO-T-Watch-Programmable-And-Networked-Open-Source-Smart-Watch-That-Interacts-With-The-Environment-As-A-Wearable-Device-p-1501125.html?cur_warehouse=GWTR&ID=233)*.

* Breakdown of Code

You might need to update some of the pins identified in the code since we have done some soldering. This file:

1.  Updates the watch to save battery (turns off the screen unless prompted)
2.  Looks to connect to WiFi
3.  Connects to an open web socket
4.  Once connected it will stream quartian location data (x, y, z, w) to the router that will then be aggregated in a raspberry pi
