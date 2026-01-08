# Purdue RI3D Project Base

This is a robot project base intended to be used for fast prototyping with
many plug-and-play built-ins

## Features
* Plug-and-play architecture
   - Pre-built components that can be easily combined into subsystems
   - Motor agnostic components for a combatability with different motor controlers
* Simulatable
   - The ability to run the robot in simulation with high enough accuracy to test complex commands and sequences
* Effective Logging
   - Utilizes DogLog to have subsystems cleanly log their own key values
* Unit Management
   - Usage of WPILib Units Library to easily and clearly store values in whatever desired units

## Examples
* ExampleArm
   - Represents an end effector mounted to a rotating arm on an elevator
   - All motors are SparkMax
* ExampleClimber
   - Represents a simple linear climber (ex: Climber in a Box)
   - Powered by TalonFX motor
* ExampleIntake
   - Represents a simple 'flip down' intake with a beam break to detect game piece acquisition
   - Pivot is TalonFX while roller is SparkMax
* ExampleShooter
   - Represents a flywheel on adjustable pivot with a feeder to feed in game pieces and a beam break to detect when game piece is close to contacting flywheel
   - All motors are TalonFX

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

## Current State
NOT ALL FEATURES HAVE BEEN EXTENSIVELY TESTED YET. UNDISCOVERED BUGS MAY STILL EXIST.
 * IOs
   - TalonFX -- Working in simulation
   - TalonFXS -- Untested
   - SparkMax -- Working in simulation
   - SparkFlex -- Untested
   - DigitalInIO -- Untested
   - DigitalBooleanSupplierIO -- Working in simulation
 * Components
   - MotorComponent -- Working in simulation
   - ServoMotorComponent -- Working in simulation
   - HomingServoMotorComponent -- Working in simulation
   - FlywheelMotorComponent -- Working in simulation
   - DigitalIOComponent -- Working in simulation
