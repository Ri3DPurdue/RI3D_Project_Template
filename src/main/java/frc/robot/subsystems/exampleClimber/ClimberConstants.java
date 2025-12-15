package frc.robot.subsystems.exampleClimber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.component.ServoMotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;
import frc.lib.io.motor.ctre.TalonFXIOSim;
import frc.lib.io.motor.setpoints.PositionSetpoint;
import frc.lib.mechanismSim.LinearSim;
import frc.lib.mechanismSim.SimObject;
import frc.lib.util.UnitsUtil.DistanceAngleConverter;
import frc.robot.IDs;
import frc.robot.Robot;

public class ClimberConstants {
    // Create a converter to convert linear distances of the climber into rotations of the motor
    public static final DistanceAngleConverter converter = new DistanceAngleConverter(
            Units.Inches.of(2.0) // Diameter of pulley
            .plus(Units.Inches.of(0.25)) // Diameter of rope
            .div(2.0) // Divide by 2 to get radius
        );

    // Epsilon threshold is distance that is considered "close" for internal methods and wait commands. Lower value is higher required accuracy
    public static final Distance epsilonThreshold = Units.Centimeters.of(2.0);
    
    // Gearing is a 5 to 1 reduction followed by another 5 to 1 reduction
    public static final double gearing = (5.0 / 1.0) * (5.0 / 1.0);
    
    // Constraints of the system's movement (hard stops, potential interferences, soft limits, etc.)
    public static final Distance minDistance = Units.Meters.of(0.0);
    public static final Distance maxDistance = Units.Meters.of(1.2);
    
    // Notable points for system
    public static final Distance extendDistance = Units.Meters.of(1.08);
    public static final Distance stowDistance = minDistance;
    public static final Distance pullDistance = Units.Inches.of(0.23);
    
    // Setpoints for notable points
    public static final PositionSetpoint extendSetpoint = new PositionSetpoint(converter.toAngle(extendDistance));
    public static final PositionSetpoint stowSetpoint = new PositionSetpoint(converter.toAngle(stowDistance));
    public static final PositionSetpoint pullSetpoint = new PositionSetpoint(converter.toAngle(pullDistance));
    
    // Information about motors driving system
    public static final DCMotor motor = DCMotor.getKrakenX60(2); // Only needed for sim

    /**
     *  Gets the final component for the system
     */ 
    public static final ServoMotorComponent<TalonFXIO> getComponent() {
        TalonFXIO io = getMotorIO();
        io.overrideLoggedUnits(converter.asAngleUnit(Units.Inches), converter.asAngularVelocityUnit(Units.InchesPerSecond), Units.Celsius);
        return new ServoMotorComponent<TalonFXIO>(getMotorIO(), converter.toAngle(epsilonThreshold), converter.toAngle(stowDistance));
    }

    /**
     * Gets a MotorIO for the system, returning a real one when actually running and a simulated one when running the simulation.
     */
    @SuppressWarnings("unchecked")
    public static final TalonFXIO getMotorIO() {
        return Robot.isReal() 
            ? new TalonFXIO(
                IDs.CLIMBER_MAIN.id,
                new CANBus(IDs.CLIMBER_MAIN.bus),
                getMainConfig(),
                Pair.of(IDs.CLIMBER_FOLLOWER.id, false)
                )
            : new TalonFXIOSim(
                IDs.CLIMBER_MAIN.id,
                new CANBus(IDs.CLIMBER_MAIN.bus),
                getMainConfig(),
                getSimObject(),
                Pair.of(IDs.CLIMBER_FOLLOWER.id, false)
            );
    }

    /** 
     * Get the configuration of the main motor
     */ 
    public static final TalonFXConfiguration getMainConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.1;
        config.Slot0.kD = 0.15;

        return config;    
    }

    /**
     * Gets an object to represent the system when running simulation
     */
    public static final SimObject getSimObject() {
        ElevatorSim system = 
            new ElevatorSim(
                motor, 
                gearing, 
                0.01, 
                0.2, 
                minDistance.in(Units.Meters), 
                minDistance.in(Units.Meters), 
                false,
                stowDistance.in(Units.Meters), 
                0.0, 0.0);
        return new LinearSim(system, converter);
    }
}
