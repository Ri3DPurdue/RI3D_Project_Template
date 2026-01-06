package frc.robot.subsystems.exampleArm;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.component.HomingServoMotorComponent.HomingConfig;
import frc.lib.component.HomingServoMotorComponent;
import frc.lib.io.motor.rev.SparkBaseIO;
import frc.lib.io.motor.rev.SparkBaseSimIO;
import frc.lib.io.motor.setpoints.PositionSetpoint;
import frc.lib.mechanismSim.LinearSim;
import frc.lib.mechanismSim.SimObject;
import frc.lib.util.ConfigUtil;
import frc.lib.util.UnitsUtil.DistanceAngleConverter;
import frc.robot.IDs;
import frc.robot.Robot;

public class ElevatorConstants {
    // Create a converter to convert linear distances of the climber into rotations of the sprocket
    public static final DistanceAngleConverter converter = new DistanceAngleConverter(
            Units.Inches.of(2.0) // Effective radius of a sprocket driving elevator on a chain
        );

    // Epsilon threshold is distance that is considered "close" for internal methods and wait commands. Lower value is higher required accuracy
    public static final Distance epsilonThreshold = Units.Inches.of(0.5);

    // Gearing is a 5 to 1 reduction followed by a 3 to 1 reduction
    public static final double gearing = (5.0 / 1.0) * (3.0 / 1.0);

    // Constraints of the system's movement (hard stops, potential interferences, soft limits, etc.)
    public static final Distance minHeight = Units.Inches.of(0.0);
    public static final Distance maxHeight = Units.Inches.of(12.0);

    // Notable points for system
    public static final Distance scoreHeight = Units.Inches.of(10.0);
    public static final Distance stowHeight = minHeight;

    // Setpoints for notable points
    public static final PositionSetpoint scoreSetpoint = new PositionSetpoint(converter.toAngle(scoreHeight));
    public static final PositionSetpoint stowSetpoint = new PositionSetpoint(converter.toAngle(stowHeight));

    // Information about motors driving system
    public static final MotorType motorType = MotorType.kBrushless; // Only needed for Sparks 
    public static final DCMotor motor = DCMotor.getNeo550(2); // Only needed for sim

    // Gets the final component for the system
    public static final HomingServoMotorComponent<SparkBaseIO> getComponent() {
        SparkBaseIO io = getMotorIO();
        io.overrideLoggedUnits(converter.asAngleUnit(Units.Inches), converter.asAngularVelocityUnit(Units.InchesPerSecond), Units.Celsius);
        return new HomingServoMotorComponent<SparkBaseIO>(io, converter.toAngle(epsilonThreshold), converter.toAngle(stowHeight), getHomingConfig());
    }

    /**
     * Gets a MotorIO for the system, returning a real one when actually running and a simulated one when running the simulation.
     */
    @SuppressWarnings("unchecked")
    public static final SparkBaseIO getMotorIO() {
        return Robot.isReal() 
            ? new SparkBaseIO(
                MotorType.kBrushless, 
                getMainConfig(), 
                IDs.ARM_ELEVATOR_MAIN.id, 
                Pair.of(IDs.ARM_ELEVATOR_FOLLOWER.id, false)
                )
            : new SparkBaseSimIO(
                getSimObject(),
                motor,
                MotorType.kBrushless, 
                getMainConfig(), 
                IDs.ARM_ELEVATOR_MAIN.id, 
                Pair.of(IDs.ARM_ELEVATOR_FOLLOWER.id, false)
            );
    }

    /*
     * Get the configuration of the main motor
     */ 
    public static final SparkBaseConfig getMainConfig() {
        SparkMaxConfig config = ConfigUtil.getSafeMaxConfig(gearing);
        ConfigUtil.withSoftLimits(config, converter.toAngle(maxHeight), converter.toAngle(minHeight));
        config.closedLoop
            .p(0.2)
            .d(0.0);
        return config;    
    }

    /**
     * Gets the homing configuration for the component
     */
    public static final HomingConfig getHomingConfig() {
        HomingConfig config = new HomingConfig();

        config.homePosition = converter.toAngle(stowHeight);
        config.homeSetpoint = stowSetpoint;
        config.homingDebouce = Seconds.of(0.1);
        config.homingVelocity = converter.toAngle(Centimeters.per(Seconds).of(0.5));
        config.homingVoltage = Volts.of(-2.0);

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
                5.0, 
                0.08, 
                minHeight.in(Units.Meters), 
                maxHeight.in(Units.Meters), 
                false,
                stowHeight.in(Units.Meters), 
                0.0, 0.0);
        return new LinearSim(system, converter);
    }
}
