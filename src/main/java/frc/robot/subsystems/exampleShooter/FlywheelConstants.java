package frc.robot.subsystems.exampleShooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.component.FlywheelMotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;
import frc.lib.io.motor.ctre.TalonFXIOSim;
import frc.lib.io.motor.setpoints.IdleSetpoint;
import frc.lib.io.motor.setpoints.VelocitySetpoint;
import frc.lib.io.motor.setpoints.VoltageSetpoint;
import frc.lib.mechanismSim.RollerSim;
import frc.lib.mechanismSim.SimObject;
import frc.robot.IDs;
import frc.robot.Robot;

public class FlywheelConstants {
    // Epsilon threshold is velocity that is considered "close" for internal methods and wait commands. Lower value is higher required accuracy
    public static final AngularVelocity epsilonThreshold = Units.RPM.of(100);
    
    // Gearing is a 48 to 40 reduction
    public static final double gearing = (48.0 / 40.0);

    // Notable points for system
    public static final AngularVelocity shotVelocity = Units.RPM.of(2000.0);
    public static final Voltage unjamVoltage = Units.Volts.of(-8.0);

    // Setpoints for notable points
    public static final VelocitySetpoint shotSetpoint = new VelocitySetpoint(shotVelocity);
    public static final VoltageSetpoint feedSetpoint = new VoltageSetpoint(unjamVoltage);
    public static final IdleSetpoint idleSetpoint = new IdleSetpoint();

    // Information about motors driving system
    public static final DCMotor motor = DCMotor.getKrakenX60(2); // Only needed for sim

    /**
     *  Gets the final component for the system
     */ 
    public static final FlywheelMotorComponent<TalonFXIO> getComponent() {
        return new FlywheelMotorComponent<TalonFXIO>(getMotorIO(), epsilonThreshold);
    }

    /**
     * Gets a MotorIO for the system, returning a real one when actually running and a simulated one when running the simulation.
     */
    @SuppressWarnings("unchecked")
    public static final TalonFXIO getMotorIO() {
        return Robot.isReal() 
            ? new TalonFXIO(
                IDs.SHOOTER_FLYWHEEL_MAIN.id,
                new CANBus(IDs.SHOOTER_FLYWHEEL_MAIN.bus),
                getMainConfig(),
                Pair.of(IDs.SHOOTER_FLYWHEEL_FOLLOWER.id, false)
                )
            : new TalonFXIOSim(
                IDs.SHOOTER_FLYWHEEL_MAIN.id,
                new CANBus(IDs.SHOOTER_FLYWHEEL_MAIN.bus),
                getMainConfig(),
                getSimObject(),
                Pair.of(IDs.SHOOTER_FLYWHEEL_FOLLOWER.id, false)
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

    public static final SimObject getSimObject() {
        FlywheelSim system = 
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    motor, 
                    0.01, 
                    gearing),
                motor
            );
        return new RollerSim(system);
    }
}
