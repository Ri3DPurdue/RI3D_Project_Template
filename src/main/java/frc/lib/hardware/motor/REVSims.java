package frc.lib.hardware.motor;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.lib.io.motor.MotorOutputs;
import frc.lib.sim.SimObject;

public class REVSims {
    public static abstract class SparkMaxSimIO extends SparkMaxIO {
        private SparkMaxSim simMotor;
        private SimObject simObject;

        protected SparkMaxSimIO(SimObject simObject, DCMotor motor, MotorType type, int mainMotor, int... followers) {
            super(type, mainMotor, followers);

            this.simMotor = new SparkMaxSim(
                main.motor,
                motor
            );

            this.simObject = simObject;
        }

        @Override
        public void periodic() {
            super.periodic();
            MotorOutputs outputs = getOutputs()[0];
            
            simObject.setVoltage(outputs.statorVoltage);
            simObject.update();

            // Radians per second
            double velocity = simObject.getVelocity() * distanceFactor;


            simMotor.iterate(
                Units.radiansPerSecondToRotationsPerMinute(velocity),
                RoboRioSim.getVInVoltage(),
                0.02
            );
        }
    }

    public static abstract class SparkFlexSimIO extends SparkFlexIO {
        private SparkFlexSim simMotor;
        private SimObject simObject;

        protected SparkFlexSimIO(SimObject simObject, DCMotor motor, MotorType type, int mainMotor, int... followers) {
            super(type, mainMotor, followers);

            this.simMotor = new SparkFlexSim(
                main.motor,
                motor
            );

            this.simObject = simObject;
        }

        @Override
        public void periodic() {
            super.periodic();
            MotorOutputs outputs = getOutputs()[0];
            
            simObject.setVoltage(outputs.statorVoltage);
            simObject.update();

            // Radians per second
            double velocity = simObject.getVelocity() * distanceFactor;


            simMotor.iterate(
                Units.radiansPerSecondToRotationsPerMinute(velocity),
                RoboRioSim.getVInVoltage(),
                0.02
            );
        }
    }

    public static class NeoSim extends SparkMaxSimIO {    
        public NeoSim(SimObject simObject, int mainMotor, int... followers) {
            super(
                simObject,
                DCMotor.getNEO(1 + followers.length),
                MotorType.kBrushless,
                mainMotor,
                followers
            );
        }
    }

    public static class Neo550Sim extends SparkMaxSimIO {
        public Neo550Sim(SimObject simObject, int mainMotor, int... followers) {
            super(
                simObject,
                DCMotor.getNeo550(1 + followers.length),
                MotorType.kBrushless,
                mainMotor,
                followers
            );
        }
    }

    public static class VortexSim extends SparkFlexSimIO {
        public VortexSim(SimObject simObject, int mainMotor, int... followers) {
            super(
                simObject, 
                DCMotor.getNeoVortex(1 + followers.length),
                MotorType.kBrushless,
                mainMotor,
                followers
            );
        }
    }
}
