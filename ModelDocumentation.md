# Path Planner Model Documentation

> This repository contains the work for the Path Planning project in the Self-Driving Car Engineer Nanodegree Program of Udacity.

## Running

See [README](./README.md) for details on running the project.

## Implementation

The implementation of the Planner can be found from `./src/Planner.cpp` and its header file `./include/planner/Planner.hpp` the includes directory also contains the given `./include/helpers.h` and `./include/spline.h` headers. The Planner is used in `./app/main.cpp` where the data is handled to and from the simulation.

### Update car state

After a Planner object is initialized with data from the map it will need to be updated with the data from the car, this happends in the planner `update` function on each simulator update to the WebSocket server.
Here we simply update the car state and check if there was data from a previously planned path, this old path data can then be used to extend the new path from in order to have smooth trajectories.

### Track other traffic

When the car state has been updated, we start checking for other members on the road which might be in front, behind or left or right of our car. We do this so that the planner can make safe decisions regarding speed and lane changes. This is done within the `track` function. The track function checks each traffic member driving in the same side of the road as our simulated car. For each member car it will check from its Frenet coordinates in which lane it is and if it is in proximity of our car. With this information, our cars target speed and target lane is updated.

The following figure depicts proximity window with respect to our car (x).

```bash
# Proximity means that there is at least one car within car_s +/- SAFE_DISTANCE.
        | 0 |  1 | 2 | lane numbers
+ prox  |   |    |   | car_s + SAFE_DISTANCE
        |   |    |   |
        |   |  x |   | car_s
        |   |    |   |
- prox  |   |    |   | car_s - SAFE_DISTANCE

```

With the new information about which lane to follow and at which speed we can start to plan a new path.

### Plan new path

Once we have all the needed tracking information, path planning can start calculating a new path. This is done in the `plan` function of the planner.
With the help of previously found target lane and speed a new path will be build upon previous coordinates or our current car location (this depends on how much the previous path waypoints have ben traversed in the simulator). A few point are chosen for the new path depending on lat know location and lane information. Together with the [spline library](https://kluge.in-chemnitz.de/opensource/spline/) new waypoints are interpolated and send back to the simulator. Using the spline api to interpolate new waypoints minimizes jerk


## Checklist

- [x] The code compiles correctly.
- [x] The car is able to drive at least 4.32 miles without incident.
- [x] The car drives according to the speed limit.
- [x] Max Acceleration and Jerk are not Exceeded.
- [ ] Car does not have collisions.
- [x] The car stays in its lane, except for the time between changing lanes.
- [x] The car is able to change lanes.
- [x] There is a reflection on how to generate paths.

## Resources
