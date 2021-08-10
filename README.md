# Summer2021-TI-RSLK-Max
**Obstacle avoiding and navigation robot**

This project consists of a TI-RSLK Max kit which is used to create a simple obstacle avoiding and path navigating robot. The robot sets out on a straight path to its destination which is a given length away from the starting point. If it does not encounter any obstacle, the robot simply goes to the destination and stops there. However, if the robot encounters any obstacle on the path, it will perform a manuever to change its location away from the obstacle and recalculates the new path and gets to its final destination.


#**IMPORTANT**: In your arduino software: 
1. Add the URL http://s3.amazonaws.com/energiaUS/packages/package_energia_index.json to your File > Preferences > Additional Board Manager URLs. Also check the box for show line numbers if you like that and adjust your font size if you would like it bigger.
2. Go to Tools > Boards > Boards Manager and scroll to the bottom of the list with Energia MSP432 and install 5.29.1

Note: this step takes a few minutes, leave time for the install between 10-20 minutes.

3. After the install completes, You need to select the board and COM port in Arduino IDE. Go to Tool > Boards and now you should see the "Energia MSP432 Red Boards" > "Red LaunchPad MSP432P401R EMT" and make sure this is selected.
