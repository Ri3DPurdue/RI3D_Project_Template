package frc.lib.hardware.motor;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.lib.sim.SimObject;
import frc.lib.io.motor.FollowerConfig;
import frc.lib.io.motor.MotorOutputs;

public abstract class SparkBaseSimIO extends SparkBaseIO {
    private SparkSim simMotor;
    private SimObject simObject;

    public SparkBaseSimIO(
        SimObject simObject,
        DCMotor motor,
        MotorType motorType,
        int mainID,
        SparkBaseConfig config,
        FollowerConfig... followers
    ) {
        super(motorType, mainID, config, followers);
        
        this.simMotor = new SparkSim(this.main.motor, motor);
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

