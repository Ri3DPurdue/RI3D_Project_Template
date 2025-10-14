package frc.lib.hardware.motor;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import frc.lib.sim.SimObject;
import frc.lib.io.motor.MotorOutputs;


public class SparkBaseSimIO extends SparkBaseIO {
    private SparkSim simMotor;
    private SimObject simObject;

    @SuppressWarnings("unchecked")
    public SparkBaseSimIO(
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
        Time deltaTime = simObject.update();

        simMotor.iterate(
            simObject.getVelocity().in(Units.RadiansPerSecond),
            12,
            deltaTime.in(Units.Seconds)
        );
    }
}

