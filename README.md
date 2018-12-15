# CarND_Project11_PathPlanning

Solution for Udacity Self driving car Nano degree eleventh project: Path Planning Project.

---

## Objective

Implement a Path planning algorithm controling a car traversing around a simulated highway in Udacity simulator.

---

### Reflection

This project allows for understanding the high level reasoning, abstractions and data flow needed to achieve a human-like driving behaviour.The udacity simulator provides a simulated interactive car on a highway, it also provides traffic with random behaviour to simulate real life scenario. The path planning algroithm should observe other cars around the vehicle and plan a safe path maximizing speed without risking discomfort of onboard passengers.

---

### Prelude

I have decided, with help of the lessons instructions, to divide the problem to the following steps:

  * Environment Awarness (AKA Prediction)
      In this phase I scan the output of the lower layer sensors fused together to represent valid targets around the vehicle.
      This phase is the most critical section in my opnion as it allows for more complex path and trajectoris later on.
  * Path Planning
      Based on the current state of the environment as cars around, their speed and behaviour, We can Choose a prefered action that will maximizie our chance of a safe driving trip while maximizng vehicle velocity.
  * Trajectory generation
      After selection the safest action, we need to generate a set of way points that can be passed to lower layer actuators to try and perform and execute the planned maneuver.

---

### Environment Awarness (AKA Prediction)

The on board car sensors perform measuerments to sense the surronding environment and the output is fused and filtered to find only valid and relevant targets to our path. This complete process is abstracted and a single list of targets and their properties is provided as input to the path planning algorithm.

There exists multiple schemas to process this list and generate the needed information for next stage, The one I settled for is:
  * Implement 3 independent c++ vectors, each one represnt the detected cars in a certain lane.
  * Categorize each detected target according to it's lane and only push it to the corresponding vector if:
    * Infront of vehicle with seperation gap less than 36m ( value derivation will be discussed in Trajectory generation section)
    * Directly next to me.
    * Behind of me with seperation gap less than 15m
This schema offered the most flexibility for me allowing for more complex scenarios later on, The targets in each lane can be sorted based on the seperation gap allowing to select the most immediate threats and the filter out non-releveant targets.

---

### Path Planning

In this phase we need to output:
  * Steering action, Should we maintain our current lane or do a lane change and if so to which lane.
  * Acceleration action, should we maintain our current speed, increase, decrease and by what amount.

I have tried multiple planning schemas to ensure the selection of the best action during driving session, the one I setteled one is shown below.
In our project, our car can exist in one of 3 states:
  * Lane 1: | Car |     |     |
      We have only two safe options keep moving in lane 1 or switch to lane 2.
      * If No cars are found ahead of us on lane 1 then we should maintain moving while trying to maximize our speed.
      * If cars are found ahead of us on lane 1 but lane 2 is empty, then we can switch to lane 2 but need to decrease our speed during lane switch.
      * If cars are found in both lane 1 and lane 2 then we only decrease our speed to avoid collision and keep scanning till we find a safe gap that we can use to execute a lane switch
  * Lane 2: |     | Car |     |
      We have three safe options keep moving in lane 2 or switch to lane 1 or 3.
      * If No cars are found ahead of us on lane 2 then we should maintain moving while trying to maximize our speed.
      * If cars are found ahead of us on lane 2 but lane 1 is empty, then we can switch to lane 1 but need to decrease our speed during lane switch.
      * If cars are found ahead of us on lane 2 but lane 3 is empty, then we can switch to lane 3 but need to decrease our speed during lane switch.
      * If cars are found in all lanes then we only decrease our speed to avoid collision and keep scanning till we find a safe gap that we can use to execute a lane switch
  * Lane 3: |     |     | Car |
      We have only two safe options keep moving in lane 3 or switch to lane 2.
      It is basically the same scenario as lane 1 but mirrored.

One thing we need to take care of is that due to periodicity of the algorithm (5Hz, 200ms), during a lane change the algorithm will execute multiple times which may cause some issues as deciding to switch to a new while actually executing a lane manoeuvre and this may lead to some erratic behaviour by the vehicle.

I have implemented flag to indiicate that we are actually executing a lane switch and as such no new action should be planned till it is finished, The lane switch is finished when the car is nearly in the middle of the new lane.

---

### Trajectory generation

In this phase we need to provide a set of way points to the vehicle actuates that actually execute the action decided in previous phase.
This is done using spline library, by setting a set of anchor points as guides and evaluting the desired points to obtain x and y coordinates for the new points.

A helpful tip from the lessons is to not flush the previous calculated points but rather use them to smooth out the path and replacing only the consumed points, also decreasing computational bandwidth in the process.

A seperation gap was chosen of 36m, The maximum speed limit is 50mph which translate to 22.3 meter per second, so the car would travel 36m in around 1.6 seconds, this allows us to predict the future movememnts for nearly one and half second which seems reasonble. Not too short to be useful and not to long to be dangerous.

Two acceleration/decelration values were chosen 0.1 and 0.05 mile per hour per hour. The accelaration schema is as follows:

If critical condition: (all path blocked) --> Apply full decelration value.
If less critical situation: (lane switch) --> Apply light deceleration value.
If accelerating at low speed --> Apply full acceleration value.
If accelerating at high speed --> Apply light acceleration value. (to decrease possibility of crossing the speed limt)

---


