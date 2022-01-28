## Reflection

In this section, I am describing the code model for generating paths. 

### Structure

A single header file named `Vehicle.h` was created for our solution. The solution is designed in such a way that the Car is an object which will interact with the simulator. 

### Initialization

The car is initialized in `line 59` of `main.cpp`, 
```
 Vehicle ego(1, 0, 0, 0, 0, 0, 0);
 ```
 It is initializing the `Vehicle` object named `ego` with `lane=1` , Cartesian co-ordinates `(0, 0)`, frenet coordinates `(0, 0)` , `yaw=0`. As defined in `line 18` of `Vehicle.h`.

 The map waypoints are saved using `set_map_waypoints` method in `line 60`

### Setting the parameters per iteration

In `Line 110-114` the parameters of the car at each iteration along with the sensor fusion data from the simulator are set using the `set_*` methods.

### Lane Cost

The way this solution was approached was that the car measures cost of each lane at each iteration. The cost is defined as 
```
if lane is the same lane
    if the distance from the next car is less than 30 meters
        cost = - abs(distance)
    else
        cost = 0
else
    if the distance from behind and next cars is less than 15 meters
        cost = -abs(distance)
    else cost = 0
```

The method name is `get_lane_cost()` which is called in `Line 135` of `main.cpp` . The method is defined in `Line 129-148` of `vehicle.h` for all of the lanes. Cost per lane is calculated in `Line 150-187` of `vehicle.h`. The costs of each lane is saved in the `map` member of the `Vehicle` object named `lane_cost`

### Choosing the appropriate behavior

In `Line 136-158` of `main.cpp` the behavior planning is done. The pseudocode is described below
```
If the cost of the current lane is negative
    if  the cost of the left lane is not negative
        change lane left
    else if the cost of the right lane is not negative
        change lane right
    else if all of the lanes' costs are negative
        slow down
    else
        speed up

```