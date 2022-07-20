# Data Export and WGS84 Coordinates 

This project illustrates and gives a reference code for creating a Farming Simulator 19 script to extract raw data regarding the dynamics of the driven vehicle,
while implementing an algorithm to bring real life WG84 coordinates into the simulator API given a chosen reference point the user inputs in the script. 
The data gathered can be further processed using the Matlab scripts provided through an ad-hoc developed OOP logic. 

### Vehilce Dynamics Parameters 

In the game a negative NED (North - East - Down) reference system for the coordinates is employed. Those, togethere with the vehicle dynamics related parameters are written into a `txt` file every FPS. In the tests run the FPS rate was always fixed to 60 



| Symbol | Name / Explanation | 
| --------------- | --------------- | 
| x | NED x  | 
| y| NED y | 
| z | NED z  | 
| vx | NED x velocity|
| vy | NED y velocity |
| vz | NED z velocity |
| rpmFR | Front right wheel rpm| 
| rpmFL | Front left wheel rpm |
| rpmRR | Rear right wheel rpm|
| rpmRL | Rear left wheel rpm |
| ThetaX | Pitch angle |
| ThetaY | Yaw angle |
| ThetaZ | Roll angle |
| alphaFR | Front right wheel angle |
| alphaFL | Front left wheel angle |
| speed | vehicle speed as per in game tachometer |
| rpm | vehicle's motor rpm|
| frame | frame counter | 
| gearRatio | current gear ratio|
| steer | steering angle|
| steer_input | steering wheel input |
| latitude | WGS84 format|![wheelsRPM](https://user-images.githubusercontent.com/101090050/179984136-67b8058a-63a4-480c-b1a6-11a680703526.png)

| longitude | WGS84 format |
| height | WGS84 format| 


### Matlab results examples 

After extracting the data from the game API it is possible to process and plot them using the matlab scripts provided. 
Below several examples of the results of the plotting can be found.

![wheelsRPM](https://user-images.githubusercontent.com/101090050/179984203-13293e31-1499-4897-a173-484fda9abf6a.png)




![Steer](https://user-images.githubusercontent.com/101090050/179984277-674f21e8-ab9a-41c4-82c1-59a6a08f61b5.png)


![motorRPM](https://user-images.githubusercontent.com/101090050/179984301-1e775127-c5d2-4ecd-97c7-a9d8c8b62c59.png)

     
