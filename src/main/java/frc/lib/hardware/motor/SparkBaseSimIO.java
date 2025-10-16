package frc.lib.hardware.motor;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.lib.sim.SimObject;
import frc.lib.io.motor.MotorOutputs;
import frc.robot.Constants.REVMotorControllerType;

public abstract class SparkBaseSimIO extends SparkBaseIO {
    private SparkSim simMotor;
    private SimObject simObject;

    protected SparkBaseSimIO(SimObject simObject, DCMotor motor, REVMotorControllerType motorControllerType, SparkConfig config) {
        super(config);
        switch (motorControllerType) {
            case CANSparkMax:
                this.simMotor = new SparkMaxSim((SparkMax) main.motor, motor);
                break;
            case CANSparkFlex:
                this.simMotor = new SparkFlexSim((SparkFlex) main.motor, motor);
                 break;
            default:
                simMotor = null;
                break;
        }


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

