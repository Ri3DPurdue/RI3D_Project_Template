package frc.lib.hardware.motor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorOutputs;

import static com.revrobotics.spark.SparkBase.ControlType.*;

public class SparkMaxIO extends MotorIO {
    /**
     * Inner class for exploding a Spark Max motor controller
     * Basically, I don't want to have to call a method to get the PID controller
     * or the relative encoder every single time I want to get them
     */
    private static class Exploded {
        public final SparkMax motor;
        public final SparkClosedLoopController controller;
        public final RelativeEncoder encoder;

        public Exploded(
            int id, MotorType type
        ) {
            this.motor = new SparkMax(id, type);
            this.controller = motor.getClosedLoopController();
            this.encoder = motor.getEncoder();
        }
    }
    
    protected final Exploded main;
    protected final Exploded[] followers;

    protected SparkMaxIO(MotorType type, int mainMotor, int... followers) {
        super(followers.length);
        
        main = new Exploded(mainMotor, type);

        this.followers = new Exploded[followers.length];

        for (int i = 0; i < followers.length; i++) {
            this.followers[i] = new Exploded(followers[i], type);

            SparkMaxConfig config = new SparkMaxConfig();
            config.follow(mainMotor);

            this.followers[i].motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    /**
     * helper method for loading the data from a motor into the outputs
     * @param controller
     * @param outputs
     */
    private static void loadOutputs(Exploded controller, MotorOutputs outputs) {
        double output = controller.motor.getAppliedOutput();

        outputs.statorCurrent = controller.motor.getOutputCurrent();
        outputs.supplyCurrent = outputs.statorCurrent * output;

        outputs.supplyVoltage = controller.motor.getBusVoltage();
        outputs.statorVoltage = outputs.supplyVoltage * output;

        outputs.position = controller.encoder.getPosition();
        outputs.velocity = controller.encoder.getVelocity();
        
        outputs.temperatureCelsius = controller.motor.getMotorTemperature();
    }

    @Override
    protected void updateOutputs(MotorOutputs[] outputs) {
        loadOutputs(main, outputs[0]);

        for (int i = 0; i < followers.length; i++) {
            loadOutputs(followers[i], outputs[i + 1]);
        }
    }

    @Override
    protected void setVoltage(double voltage) {
        main.controller.setReference(voltage, kVoltage);
    }

    @Override
    protected void setCurrent(double current) {
        main.controller.setReference(current, kCurrent);
    }

    @Override
    protected void setPosition(double position) {
        main.controller.setReference(position, kPosition);
    }

    @Override
    protected void setVelocity(double velocity) {
        main.controller.setReference(velocity, kVelocity);
    }
    
    @Override
    protected void setProfiledPosition(double position) {
        main.controller.setReference(position, kMAXMotionPositionControl);
    }

    @Override
    protected void setPercentage(double percentage) {
        main.controller.setReference(percentage, kDutyCycle);
    }

    @Override
    protected void setIdle() {
        setVoltage(0);
    }
}
