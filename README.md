# timewarp

Simulation of moving and accelerating objects in 3+1D special relativity.

Includes interatial motion, longitudinal acceleration (general acceleration planned),
clocks and sending, receiving and recording light pulses.
Able to simulate clocks in accelerating rockets and the twin paradox.

Goal: Detecting collisions (e.g. for ladder paradox) and measuring frequency shifts.


Principles:

* Uses natural units (c=1): You have to convert to your preferred system yourself.
* The simulation advances the time of the inertial origin and actions can query this state (but you can transform to any other reference frme easily).

Code examples:

```java
var timewarp = TimeWarp()
// add an object at the origin (V3_0) flying at half the speed of light in x direction
Obj rocket = new Obj("Rocket");
timewarp.addObj(rocket, V3_0, EX * 0.5);
// simulate up to time 1.0
timewarp.simulateTo(1.0);
System.out.println(timewarp.getTheWorld().stateOf(rocket));
```


Links to math:
* https://en.wikipedia.org/wiki/Velocity-addition_formula
* http://math.ucr.edu/home/baez/physics/Relativity/SR/velocity.html
* https://en.wikipedia.org/wiki/Acceleration_(special_relativity)
