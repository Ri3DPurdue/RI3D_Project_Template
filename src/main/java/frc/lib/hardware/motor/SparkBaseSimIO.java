package frc.lib.hardware.motor;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.sim.SimObject;
import frc.lib.io.motor.MotorOutputs;


public abstract class SparkBaseSimIO extends SparkBaseIO {
    private SparkSim simMotor;
    private SimObject simObject;

    @SuppressWarnings("unchecked")
    protected SparkBaseSimIO(
        SimObject simObject, 
        DCMotor motor, 
        MotorType type,
        SparkBaseConfig mainConfig,
        int mainMotor, 
        Pair<Integer, Boolean>... followers
    ) {
        super(type, mainConfig, mainMotor, followers);
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
            12,
            0.02
        );
    }
}

