package frc.lib.io.motor.rev;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.lib.io.motor.MotorOutputs;
import frc.lib.mechanismSim.SimObject;

/**
 * A class that represents a simulated {@link SparkBase}
 */
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
        updateSimMotor();
    }

    /**
     * Updates the internal sim motor
     */
    private void updateSimMotor() {
        Time deltaTime = simObject.update();

        simMotor.iterate(
            simObject.getVelocity().in(Units.RPM),
            12,
            deltaTime.in(Units.Seconds)
        );
    }

    @Override
    public void resetPosition(Angle position) {
        simMotor.setPosition(position.in(Rotations));
        super.resetPosition(position);
    }
}

