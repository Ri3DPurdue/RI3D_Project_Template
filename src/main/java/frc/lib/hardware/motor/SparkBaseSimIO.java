package frc.lib.hardware.motor;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.lib.sim.SimObject;
import frc.lib.io.motor.MotorOutputs;


public abstract class SparkBaseSimIO extends SparkBaseIO {
    private SparkSim simMotor;
    private SimObject simObject;

    protected SparkBaseSimIO(
    SimObject simObject, 
    DCMotor motor, 
    REVControllerType motorControllerType, 
    MotorType type, 
    int mainMotor, 
    int... followers) {
        super(type, motor, motorControllerType, mainMotor, followers);
        this.simMotor = new SparkSim(main.motor, motor);

        this.simObject = simObject;
    }

    @Override
    public void periodic() {
        super.periodic();
        MotorOutputs outputs = getOutputs()[0];
        
        simObject.setVoltage(outputs.statorVoltage);
        simObject.update();

        // Radians per second
        double velocity = simObject.getVelocity();


        simMotor.iterate(
            Units.radiansPerSecondToRotationsPerMinute(velocity),
            RoboRioSim.getVInVoltage(),
            0.02
        );
    }
}

