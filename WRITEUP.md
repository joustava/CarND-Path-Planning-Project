# Path Planner Writeup

> This repository contains the work for the Path Planning project in the Self-Driving Car Engineer Nanodegree Program of Udacity.

## Running

See [README](./README.md) for details on running the project.

## Implementation

The implementation of the Planner can be found from `./src/Planner.cpp` and its header file `./include/planner/Planner.hpp` the includes directory also contains the given `./include/helpers.h` and `./include/spline.h` headers. The Planner is used in `./app/main.cpp` where the data is handled to and from the simulation.

### Update car state

After a Planner object is initialized with data from the map it will need to be updated with the data from the car, this happends in the planner `update` function. 
Here we simply update the car state and check if there was data from a previously planned path, this old path data can then be used to extend the new path from
in order to have smooth trajectories.

### Track other traffic

When the car state has been updated, we start checking for other members on the road which might be in front or left or right of our car. We do this so that the planner can make safe decisions regarding speed and lane changes. This is done within the `track` function. The track function checks each traffic member driving in the same side of the road as our simulated car. For each member car it will check from its Frenet coordinates in which lane it is and if it is in proximity of our car. With this information, our cars target speed and target lane is updated.

```bash

```

### Plan new path

Once we have all the needed tracking information, path planning can start calculating a new path. This is done in the `plan` function of the planner.
With the help of previously found target lane and speed a new path will be build upon previous coordinates or our current car location (this depends on how much the previous path waypoints have ben traversed in the simulator). 


## Checklist

- [x] The code compiles correctly.
- [x] The car is able to drive at least 4.32 miles without incident.
- [x] The car drives according to the speed limit.
- [x] Max Acceleration and Jerk are not Exceeded. 
- [ ] Car does not have collisions.
- [x] The car stays in its lane, except for the time between changing lanes.
- [x] The car is able to change lanes.
- [ ] There is a reflection on how to generate paths.

## Resources
