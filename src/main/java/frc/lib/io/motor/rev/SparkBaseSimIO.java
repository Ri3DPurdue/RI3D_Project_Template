package frc.lib.io.motor.rev;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import frc.lib.io.motor.MotorOutputs;
import frc.lib.mechanismSim.SimObject;


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
        updateSimMotor();
    }

    @Override
    public void periodic() {
        super.periodic();
        MotorOutputs outputs = getOutputs()[0];
        
        simObject.setVoltage(outputs.statorVoltage);
        simObject.update();
    }

    public void updateSimMotor() {
        simMotor.setPosition(simObject.getPosition().in(Units.Rotations));
        simMotor.setVelocity(simObject.getVelocity().in(Units.RPM));
    }
}

