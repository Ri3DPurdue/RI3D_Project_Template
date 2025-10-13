package frc.robot.subsystems.ExampleIntake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.component.ServoMotorComponent;
import frc.lib.hardware.motor.SparkBaseIO;
import frc.lib.hardware.motor.SparkBaseSimIO;
import frc.lib.sim.PivotSim;
import frc.lib.sim.SimObject;
import frc.robot.Ports;
import frc.robot.Robot;

public class ExampleIntakeConstants {
    public static class Pivot {
        public static final Angle epsilonThreshold = Units.Degrees.of(10.0);
        public static final double gearing = 1.0;
        public static final DCMotor motor = DCMotor.getNeo550(2);
        public static final Angle minAngle = Units.Degrees.of(0.0);
        public static final Angle maxAngle = Units.Degrees.of(110.0);


        public static final ServoMotorComponent<SparkBaseIO> getPivot() {
            return new ServoMotorComponent<SparkBaseIO>(ExampleIntakeConstants.Pivot.getMotorIO(), epsilonThreshold);
        }

        @SuppressWarnings("unchecked")
        public static final SparkBaseIO getMotorIO() {
            return Robot.isReal() 
                ? new SparkBaseIO(
                    MotorType.kBrushless, 
                    getMainConfig(), 
                    Ports.INTAKE_PIVOT_MAIN.id, 
                    Pair.of(Ports.INTAKE_PIVOT_FOLLOWER.id, false)
                    )
                : new SparkBaseSimIO(
                    getSimObject(),
                    motor,
                    MotorType.kBrushless, 
                    getMainConfig(), 
                    Ports.INTAKE_PIVOT_MAIN.id, 
                    Pair.of(Ports.INTAKE_PIVOT_FOLLOWER.id, false)
                );
        }

        public static final SparkBaseConfig getMainConfig() {
            return new SparkMaxConfig();
        }

        public static final SimObject getSimObject() {
            SingleJointedArmSim system = 
                new SingleJointedArmSim(
                    motor, 
                    1.0, 
                    1.0, 
                    1.0, 
                    minAngle.in(Units.Radians), 
                    maxAngle.in(Units.Radians), 
                    false, 
                    minAngle.in(Units.Radians), 
                    0.0, 0.0);
            return new PivotSim(system);
        }

    }
}
