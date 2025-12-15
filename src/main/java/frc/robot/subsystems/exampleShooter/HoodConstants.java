package frc.robot.subsystems.exampleShooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.Pair;
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

public class HoodConstants {
    // TODO MAKE NOT IDENTICAL TO INTAKE PIVOT
    public static final Angle epsilonThreshold = Units.Degrees.of(10.0);
    public static final double gearing = 1.0;
    public static final DCMotor motor = DCMotor.getNeo550(2);

    public static final Angle minAngle = Units.Radians.of(-10.0);
    public static final Angle maxAngle = Units.Radians.of(110.0);

    public static final Angle shotAngle = Units.Radians.of(10.0);
    public static final Angle stowAngle = Units.Radians.of(110.0);

    public static final PositionSetpoint shotSetpoint = new PositionSetpoint(shotAngle);
    public static final PositionSetpoint stowSetpoint = new PositionSetpoint(stowAngle);

    public static final ServoMotorComponent<SparkBaseIO> getComponent() {
        return new ServoMotorComponent<SparkBaseIO>(getMotorIO(), epsilonThreshold, stowAngle);
    }

    @SuppressWarnings("unchecked")
    public static final SparkBaseIO getMotorIO() {
        return Robot.isReal() 
            ? new SparkBaseIO(
                MotorType.kBrushless, 
                getMainConfig(), 
                IDs.INTAKE_PIVOT_MAIN.id, 
                Pair.of(IDs.INTAKE_PIVOT_FOLLOWER.id, false)
                )
            : new SparkBaseSimIO(
                getSimObject(),
                motor,
                MotorType.kBrushless, 
                getMainConfig(), 
                IDs.INTAKE_PIVOT_MAIN.id, 
                Pair.of(IDs.INTAKE_PIVOT_FOLLOWER.id, false)
            );
    }

    public static final SparkBaseConfig getMainConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .p(0.15)
            .d(0.15);
        return config;    
    }

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
