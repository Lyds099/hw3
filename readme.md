1.how to setup and run your program  
&emsp;&emsp;(1)Clone the repo: $ git clone https://github.com/Lyds099/hw3.git  
&emsp;&emsp;(2) $ cd TFLite_Wifi_MQTT  
&emsp;&emsp;(3)Compile the program: $ sudo mbed compile --source . --source ~/ee2405/mbed-os-build/ -m B_L4S5I_IOT01A -t GCC_ARM --profile tflite.json -f
&emsp;&emsp;(4)Choose the angle in the menu by gesture: After your selection changed, the angle that you selected will be displayed on the uLCD with green background color. After you pressed the User button to confirm, the angle that you selected will be displayed on the uLCD with blue background color. Then the selected threshold angle is published through WiFi/MQTT, the mbed will be back to RPC loop.
&emsp;&emsp;(5)Tilt the angle: The tilt angle will be displayed on uLCD. If the tilt angle is over the selected threshold angle, the event and angle are published through WiFi/MQTT. After 10 tilt events, the mbed will be back to RPC loop.

2.what are the results  
&emsp;&emsp;seletion(uLCD): https://github.com/Lyds099/hw3/blob/master/IMG_1624.jpeg?raw=true  
&emsp;&emsp;tilt angle(uLCD): https://github.com/Lyds099/hw3/blob/master/IMG_1626.jpeg?raw=true 
&emsp;&emsp;seletion(command line): https://github.com/Lyds099/hw3/blob/master/Screen%20Shot%202021-05-12%20at%2011.28.06%20AM.png?raw=true 
&emsp;&emsp;tilt angle(command line): https://github.com/Lyds099/hw3/blob/master/Screen%20Shot%202021-05-12%20at%2011.24.10%20AM.png?raw=true