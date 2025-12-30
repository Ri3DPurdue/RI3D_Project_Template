package frc.robot.subsystems.exampleIntake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.component.ServoMotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;
import frc.lib.io.motor.ctre.TalonFXIOSim;
import frc.lib.io.motor.setpoints.*;
import frc.lib.mechanismSim.PivotSim;
import frc.lib.mechanismSim.SimObject;
import frc.lib.util.ConfigUtil;
import frc.robot.IDs;
import frc.robot.Robot;

public class PivotConstants {
    // Epsilon threshold is distance that is considered "close" for internal methods and wait commands. Lower value is higher required accuracy
    public static final Angle epsilonThreshold = Units.Degrees.of(10.0);
    
    // Gearing is a 1 to 1
    public static final double gearing = 1.0;
    
    // Constraints of the system's movement (hard stops, potential interferences, soft limits, etc.)
    public static final Angle minAngle = Units.Degrees.of(-10.0);
    public static final Angle maxAngle = Units.Degrees.of(110.0);

    // Notable points for system
    public static final Angle deployAngle = Units.Degrees.of(-3.0);
    public static final Angle stowAngle = maxAngle;
    public static final Angle unjamAngle = Units.Degrees.of(70.0);
    
    // Setpoints for notable points
    public static final PositionSetpoint deploySetpoint = new PositionSetpoint(deployAngle);
    public static final PositionSetpoint stowSetpoint = new PositionSetpoint(stowAngle);
    public static final PositionSetpoint unjamSetpoint = new PositionSetpoint(unjamAngle);

    // Information about motors driving system
    public static final DCMotor motor = DCMotor.getFalcon500(1); // Only needed for sim

    // Gets the final component for the system
    public static final ServoMotorComponent<TalonFXIO> getComponent() {
        return new ServoMotorComponent<TalonFXIO>(getMotorIO(), epsilonThreshold, unjamAngle);
    }

    /**
     * Gets a MotorIO for the system, returning a real one when actually running and a simulated one when running the simulation.
     */
    @SuppressWarnings("unchecked")
    public static final TalonFXIO getMotorIO() {
        return Robot.isReal() 
            ? new TalonFXIO(
                IDs.INTAKE_PIVOT.id,
                new CANBus(IDs.INTAKE_PIVOT.bus),
                getMainConfig()
                )
            : new TalonFXIOSim(
                IDs.INTAKE_PIVOT.id,
                new CANBus(IDs.INTAKE_PIVOT.bus),
                getMainConfig(),
                getSimObject()
            );
    }

    /** 
     * Get the configuration of the main motor
     */ 
    public static final TalonFXConfiguration getMainConfig() {
        TalonFXConfiguration config = ConfigUtil.getSafeFXConfig(gearing);
        config.Slot0.kP = 0.1;
        config.Slot0.kD = 0.15;

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
                0.0, 
                0.0, 0.0);
        return new PivotSim(system);
    }
}
