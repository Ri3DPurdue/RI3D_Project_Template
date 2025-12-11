# Purdue RI3D Project Base

This is a robot project base intended to be used for fast prototyping with
many plug-and-play built-ins

## Features
1) Plug-and-play architecture
   - Pre-built components that can be combined into subsystems
   - Motor agnostic components for a combatability with different motor controlers
2) Simulatable
   - The ability to run the robot in simulation with high enough accuracy to test complex, high level commands and sequences
3) Effective Logging
   - Utilizes DogLog to have subsystems cleanly log their own key values
4) Unit Management
   - Usage of WPILib Units Library to easily and clearly store values in proper units

## How It's Achieved
 * Superstructure
   - Handles high level coordination between different subsystems and drivetrain

 * Component Subsystem
   - Stores motor and sensor components within a subsystem and coordinates them from internal subsystem movements

 * Component
   - Stores different types of IOs for easy utilization within a Component Subsystem

 * MotorIO
   - Abstract class extended for all used motor controllers to allow higher level features to be motor agnostic
   - Every motor controller also has simulatable versions utilizing SimObjects
      - SimObjects are simulations of mechanisms usable my a MotorIO child for simple internal simulation
   - Stores everything in mechanism units (angle of last rotating component in system such as arm, flywheel, or pulley of elevator)

 * DigitalIO
   - Abstract class extended for all different sensors that represent digital/boolean values to allow higher level features to be sensor agnostic
   - Can be easily simulated off a controller button using DigitalBooleanSupplierIO