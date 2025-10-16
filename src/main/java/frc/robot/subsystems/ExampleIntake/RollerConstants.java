package frc.robot.subsystems.ExampleIntake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.component.MotorComponent;
import frc.lib.hardware.motor.SparkBaseIO;
import frc.lib.hardware.motor.SparkBaseSimIO;
import frc.lib.io.motor.Setpoint;
import frc.lib.sim.RollerSim;
import frc.lib.sim.SimObject;
import frc.robot.Ports;
import frc.robot.Robot;

public class RollerConstants {
    public static final double gearing = 1.0;
    public static final DCMotor motor = DCMotor.getNeo550(1);

    public static final Voltage inwardsVoltage = Units.Volts.of(8.0);
    public static final Voltage spitVoltage = Units.Volts.of(-6.0);

    public static final Setpoint inwardsSetpoint = Setpoint.voltageSetpoint(inwardsVoltage);
    public static final Setpoint spitSetpoint = Setpoint.voltageSetpoint(spitVoltage);
    public static final Setpoint idleSetpoint = Setpoint.idleSetpoint();

    public static final MotorComponent<SparkBaseIO> getRoller() {
        return new MotorComponent<SparkBaseIO>(getMotorIO());
    }

    @SuppressWarnings("unchecked")
    public static final SparkBaseIO getMotorIO() {
        return Robot.isReal() 
            ? new SparkBaseIO(
                MotorType.kBrushless, 
                getMainConfig(), 
                Ports.INTAKE_ROLLERS.id
                )
            : new SparkBaseSimIO(
                getSimObject(),
                motor,
                MotorType.kBrushless, 
                getMainConfig(), 
                Ports.INTAKE_ROLLERS.id
            );
    }

    public static final SparkBaseConfig getMainConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
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
