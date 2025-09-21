package frc.lib.hardware.motor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorOutputs;

import static com.revrobotics.spark.SparkBase.ControlType.*;

public class SparkFlexIO extends MotorIO {
    private static class SparkController {
        public final SparkFlex motor;
        public final SparkClosedLoopController controller;
        public final RelativeEncoder encoder;

        public SparkController(
            int id, MotorType type
        ) {
            this.motor = new SparkFlex(id, type);
            this.controller = motor.getClosedLoopController();
            this.encoder = motor.getEncoder();
        }
    }
    
    protected final SparkController main;
    protected final SparkController[] followers;

    protected SparkFlexIO(MotorType type, int mainMotor, int... followers) {
        super(followers.length);
        
        main = new SparkController(mainMotor, type);

        this.followers = new SparkController[followers.length];

        for (int i = 0; i < followers.length; i++) {
            this.followers[i] = new SparkController(followers[i], type);

            SparkFlexConfig config = new SparkFlexConfig();
            config.follow(mainMotor);

            this.followers[i].motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    private static void loadOutputs(SparkController controller, MotorOutputs outputs) {
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

