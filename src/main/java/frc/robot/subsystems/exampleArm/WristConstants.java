package frc.robot.subsystems.exampleArm;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.component.ServoMotorComponent;
import frc.lib.io.motor.rev.SparkBaseIO;
import frc.lib.io.motor.rev.SparkBaseSimIO;
import frc.lib.io.motor.setpoints.PositionSetpoint;
import frc.lib.mechanismSim.PivotSim;
import frc.lib.mechanismSim.SimObject;
import frc.robot.IDs;
import frc.robot.Robot;

public class WristConstants {
    // Epsilon threshold is distance that is considered "close" for internal methods and wait commands. Lower value is higher required accuracy
    public static final Angle epsilonThreshold = Units.Degrees.of(10.0);
    
    // Gearing is a 6 to 1 reduction
    public static final double gearing = (6.0 / 1.0);
    
    // Constraints of the system's movement (hard stops, potential interferences, soft limits, etc.)
    public static final Angle minAngle = Units.Radians.of(-10.0);
    public static final Angle maxAngle = Units.Radians.of(110.0);
    
    // Notable points for system
    public static final Angle scoreAngle = Units.Degrees.of(10.0);
    public static final Angle stowAngle = Units.Radians.of(110.0);
    
    // Setpoints for notable points
    public static final PositionSetpoint scoreSetpoint = new PositionSetpoint(scoreAngle);
    public static final PositionSetpoint stowSetpoint = new PositionSetpoint(stowAngle);
    
    // Information about motors driving system
    public static final MotorType motorType = MotorType.kBrushless; // Only needed for Sparks    
    public static final DCMotor motor = DCMotor.getNeo550(1); // Only needed for sim

    // Gets the final component for the system
    public static final ServoMotorComponent<SparkBaseIO> getComponent() {
        return new ServoMotorComponent<SparkBaseIO>(getMotorIO(), epsilonThreshold, stowAngle);
    }

    /**
     * Gets a MotorIO for the system, returning a real one when actually running and a simulated one when running the simulation.
     */
    @SuppressWarnings("unchecked")
    public static final SparkBaseIO getMotorIO() {
        return Robot.isReal() 
            ? new SparkBaseIO(
                motorType, 
                getMainConfig(), 
                IDs.ARM_WRIST.id
                )
            : new SparkBaseSimIO(
                getSimObject(),
                motor,
                motorType, 
                getMainConfig(), 
                IDs.ARM_WRIST.id
            );
    }

    /** 
     * Get the configuration of the main motor
     */ 
    public static final SparkBaseConfig getMainConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .p(0.15)
            .d(0.15);
        return config;    
    }
    
    /**
     * Gets an object to represent the system when running simulation
     */
    public static final SimObject getSimObject() {
        SingleJointedArmSim system = 
            new SingleJointedArmSim(
                motor, 
                gearing, 
                0.01, 
                0.2, 
                minAngle.in(Units.Radians), 
                maxAngle.in(Units.Radians), 
                false,
                stowAngle.in(Units.Radians), 
                0.0, 0.0);
        return new PivotSim(system);
    }
}
