package frc.robot.subsystems.exampleShooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.component.MotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;
import frc.lib.io.motor.ctre.TalonFXIOSim;
import frc.lib.io.motor.setpoints.IdleSetpoint;
import frc.lib.io.motor.setpoints.VoltageSetpoint;
import frc.lib.mechanismSim.RollerSim;
import frc.lib.mechanismSim.SimObject;
import frc.lib.util.ConfigUtil;
import frc.robot.IDs;
import frc.robot.Robot;

public class FeederConstants {
    // Gearing is a 36 to 16 reduction
    public static final double gearing = (36.0 / 16.0);

    // Notable points for system
    public static final Voltage feedVoltage = Units.Volts.of(8.0);
    public static final Voltage intakeVoltage = Units.Volts.of(2.0);

    // Setpoints for notable points
    public static final VoltageSetpoint feedSetpoint = new VoltageSetpoint(feedVoltage);
    public static final VoltageSetpoint intakeSetpoint = new VoltageSetpoint(intakeVoltage);
    public static final IdleSetpoint idleSetpoint = new IdleSetpoint();

    // Information about motors driving system 
    public static final DCMotor motor = 
        new DCMotor( // Specs for Kraken x44 from https://docs.wcproducts.com/welcome/electronics/kraken-x44/kraken-x44-motor/overview-and-features/motor-performance because not included in DCMotor pre-provided motors 
            12.0, 
            4.05, 
            275.0, 
            1.4, 
            edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond(7530.0), 
            1 // Number of Kraken x44 motors
        ); // Only needed for sim 

    /**
     * Gets a MotorIO for the system, returning a real one when actually running and a simulated one when running the simulation.
     */
    public static final MotorComponent<TalonFXIO> getComponent() {
        return new MotorComponent<TalonFXIO>(getMotorIO());
    }

    /**
     * Gets a MotorIO for the system, returning a real one when actually running and a simulated one when running the simulation.
     */
    @SuppressWarnings("unchecked")
    public static final TalonFXIO getMotorIO() {
        return Robot.isReal() 
            ? new TalonFXIO(
                IDs.SHOOTER_FEEDER.id,
                new CANBus(IDs.SHOOTER_FEEDER.bus),
                getMainConfig()
                )
            : new TalonFXIOSim(
                IDs.SHOOTER_FEEDER.id,
                new CANBus(IDs.SHOOTER_FEEDER.bus),
                getMainConfig(),
                getSimObject()
            );
    }

    /** 
     * Get the configuration of the main motor
     */ 
    public static final TalonFXConfiguration getMainConfig() {
        TalonFXConfiguration config = ConfigUtil.getSafeFXConfig(gearing);

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
