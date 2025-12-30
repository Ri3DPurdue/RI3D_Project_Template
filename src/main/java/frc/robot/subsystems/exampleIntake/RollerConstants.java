package frc.robot.subsystems.exampleIntake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.component.MotorComponent;
import frc.lib.io.motor.rev.SparkBaseIO;
import frc.lib.io.motor.rev.SparkBaseSimIO;
import frc.lib.io.motor.setpoints.*;
import frc.lib.mechanismSim.RollerSim;
import frc.lib.mechanismSim.SimObject;
import frc.lib.util.ConfigUtil;
import frc.robot.IDs;
import frc.robot.Robot;

public class RollerConstants {
    // Gearing is a 36 to 16 reduction
    public static final double gearing = (36.0 / 16.0);

    // Notable points for system
    public static final Voltage inwardsVoltage = Units.Volts.of(8.0);
    public static final Voltage spitVoltage = Units.Volts.of(-6.0);

    // Setpoints for notable points
    public static final VoltageSetpoint inwardsSetpoint = new VoltageSetpoint(inwardsVoltage);
    public static final VoltageSetpoint spitSetpoint = new VoltageSetpoint(spitVoltage);
    public static final IdleSetpoint idleSetpoint = new IdleSetpoint();

    // Information about motors driving system
    public static final MotorType motorType = MotorType.kBrushless; // Only needed for Sparks 
    public static final DCMotor motor = DCMotor.getNeo550(1); // Only needed for sim

    /**
     * Gets a MotorIO for the system, returning a real one when actually running and a simulated one when running the simulation.
     */
    public static final MotorComponent<SparkBaseIO> getComponent() {
        return new MotorComponent<SparkBaseIO>(getMotorIO());
    }

    @SuppressWarnings("unchecked")
    public static final SparkBaseIO getMotorIO() {
        return Robot.isReal() 
            ? new SparkBaseIO(
                motorType, 
                getMainConfig(), 
                IDs.INTAKE_ROLLERS.id
                )
            : new SparkBaseSimIO(
                getSimObject(),
                motor,
                motorType, 
                getMainConfig(), 
                IDs.INTAKE_ROLLERS.id
            );
    }

    /**
     * Get the configuration of the main motor
     */
    public static final SparkBaseConfig getMainConfig() {
        SparkMaxConfig config = ConfigUtil.getSafeMaxConfig(gearing);
        return config;
    }

    /**
     * Gets an object to represent the system when running simulation
     */
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
