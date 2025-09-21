# Purdue RI3D Project Base

This is a robot project base intended to be used for fast prototyping with
many plug-and-play built-ins

## Goals
1) Plug-and-play architecture
   - Pre-built mechanisms that can be combined into subsystems
   - Motor agnostic mechanisms for a wide variety of prototyping capabilities
2) Simulatable
   - The ability to run the robot in simulation and make it as accurate to the real robot as possible
3) Loggable
   - The ability to run the robot and see what is happening to it at all points in time, even after we disable it
4) Clean and simple
   - The ability for anyone to read our code and know what is going on well enough
   - This also implies good documentation
5) Good unit management (future section)

## Non-Goals
1) Perfection
   - This is never going to be perfect. Deal with it.
2) 100% use case coverage
   - We are an RI3D team. At the end of the day, we need to prioritize quick prototyping
     rather than covering every possible use case of our code

## Units
We don't want to use the built in Units library to WPILib because we don't want to
deal with all of the allocations bogging down our RIO.<br>
So, we always return a ```double``` For our units<br>
This also means we need to be careful in how we name/document our code.<br>
This is intended to be a step in the right direction<br>

### WPILib Units
For the most part, we are going to stay away from these<br>
But, we will use the WPILib conversions set to help us keep a common convention<br>
To that point, we will also use the WPILib convention for distances, angles and coordinate systems<br>

### Distance
Unless explicitly stated otherwise, all distances are stored in doubles and measured in meters.<br>
Any conversions from a different unit should be handled by the WPILib conversion set<br>

### Angles
Unless explicitly stated otherwise, all angles are stored in Rotation2d objects, counterclockwise positive<br>

### Velocities
Unless explicity stated otherwise, all velocities are stored in doubles and measured in meters per second.<br>
And conversions from a different unit should be handled by the WPILib conversion set<br>

### Angular Velocities
Unless explicitly stated otherwise, all angular velocities are stored in doubles and measured in radians per second with
counterclockwise being positive.<br>
Any conversions from a different unit should be handled by the WPILib conversion set<br>

### Motor Inputs
Motor inputs are based on the mechanism to which they are attached, so the units are as well<br>
Take an arm. You want to set the angle of that arm from level. You would tell the ```MotorIO``` that angle and it
would handle the conversion from there<br>
Now take an elevator. You want to set the height of that elevator from zero. You would tell the ```MotorIO``` that height
and it would handle the conversion from there<br>

### Positions
Unless explicitly stated otherwise, all positions are stored in either a ```Translation2d``` or ```Pose2d``` with the
conventions therein.<br>
This means that the blue driverstation is on the left, and the red driverstation is on the right, and the field is in front
of you<br>
To your right is 0 degrees, and ahead of you is 90 degrees<br>
The corner created from the alliance wall on the blue side and the field wall parallel to 0 degrees and closest to you is 0, 0<br>

### BEANS (Best Emperically Allowed Number)
No. Not again. It was too confusing, apparently...
