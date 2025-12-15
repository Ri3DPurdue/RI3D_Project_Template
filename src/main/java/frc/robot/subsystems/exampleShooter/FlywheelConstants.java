package frc.robot.subsystems.exampleShooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.component.FlywheelMotorComponent;
import frc.lib.io.motor.rev.SparkBaseIO;
import frc.lib.io.motor.rev.SparkBaseSimIO;
import frc.lib.io.motor.setpoints.IdleSetpoint;
import frc.lib.io.motor.setpoints.VelocitySetpoint;
import frc.lib.mechanismSim.RollerSim;
import frc.lib.mechanismSim.SimObject;
import frc.robot.IDs;
import frc.robot.Robot;

public class FlywheelConstants {
    // TODO MAKE NOT IDENTICAL TO INTAKE INDEXER
    public static final AngularVelocity epsilonThreshold = Units.RPM.of(100);
    public static final double gearing = 1.0;
    public static final DCMotor motor = DCMotor.getNeo550(1);

    public static final AngularVelocity shotVelocity = Units.RPM.of(2000.0);
    public static final Voltage spitVoltage = Units.Volts.of(-6.0);

    public static final VelocitySetpoint shotSetpoint = new VelocitySetpoint(shotVelocity);
    public static final IdleSetpoint idleSetpoint = new IdleSetpoint();

    public static final FlywheelMotorComponent<SparkBaseIO> getComponent() {
        return new FlywheelMotorComponent<SparkBaseIO>(getMotorIO(), epsilonThreshold);
    }

    @SuppressWarnings("unchecked")
    public static final SparkBaseIO getMotorIO() {
        return Robot.isReal() 
            ? new SparkBaseIO(
                MotorType.kBrushless, 
                getMainConfig(), 
                IDs.INTAKE_INDEXER.id
                )
            : new SparkBaseSimIO(
                getSimObject(),
                motor,
                MotorType.kBrushless, 
                getMainConfig(), 
                IDs.INTAKE_INDEXER.id
            );
    }

    public static final SparkBaseConfig getMainConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .p(0.15)
            .d(0.1);
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
