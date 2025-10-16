package frc.lib.hardware.motor;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.lib.io.motor.BaseConfig;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorOutputs;
import frc.robot.Constants.REVMotorControllerType;

import static com.revrobotics.spark.SparkBase.ControlType.*;

public class SparkBaseIO extends MotorIO {
    public static class SparkConfig extends BaseConfig {
        public REVMotorControllerType controllerType;
        public MotorType motorType = MotorType.kBrushless;
    }

    /**
     * Inner class for exploding a Spark Max motor controller (works since both SparkMax and SparkFlex extend the same type)
     * Basically, I don't want to have to call a method to get the PID controller
     * or the relative encoder every single time I want to get them
     * @param motor Either the Spark Max or the Spark Flex
     * @param SparkClosedLoopController the REV controller to be used
     * @param RelativeEncoder The encoder of the motor controller
     */
    static class Exploded {
        public final SparkBase motor;
        public final SparkClosedLoopController controller;
        public final RelativeEncoder encoder;

        public Exploded(
            int id, MotorType type, REVMotorControllerType sparkType
        ) {
            switch (sparkType) {
                case CANSparkFlex:
                    this.motor = new SparkFlex(id, type); 
                    break;
                case CANSparkMax:
                    this.motor = new SparkMax(id,type);
                    break;

                default:
                    motor = null;
                    break;
            }
            
            this.controller = motor.getClosedLoopController();
            this.encoder = motor.getEncoder();
        }
    }
    
    protected final Exploded main;
    protected final Exploded[] followers;
    /**
     * Creates a sparkBaseIO
     * @param type Whether the motor is brushed or brushless.
     * @see com.revrobotics.spark.SparkLowLevel.MotorType
     * @param controlledMotor The motor(s) being controlled
     * @param sparkType Enum that indicates whether the motor controller is a spark max or a spark flex
     * @param mainMotor The id of the main motor
     * @param followers The id of any following motors
     */
    protected SparkBaseIO(SparkConfig config) {
        super(config);
        
        main = new Exploded(config.main.canID, config.motorType, config.controllerType);

        SparkBaseConfig sparkConfig = null;

        switch (config.controllerType) {
            case CANSparkMax:
                sparkConfig = new SparkMaxConfig();
                break;

            case CANSparkFlex:
                sparkConfig = new SparkFlexConfig();
                break;
        }

        sparkConfig.closedLoop.pidf(
            config.pid.p,
            config.pid.i,
            config.pid.d,
            config.pid.f
        );

        sparkConfig.closedLoop.maxMotion
            .maxAcceleration(config.pid.profile.maxAcceleration)
            .maxVelocity(config.pid.profile.maxVelocity);

        sparkConfig.inverted(config.main.inverted);

        this.main.motor.configure(
            sparkConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        sparkConfig.smartCurrentLimit(config.currentLimit);

        this.followers = new Exploded[config.followers.length];

        for (int i = 0; i < followers.length; i++) {
            this.followers[i] = new Exploded(config.followers[i].canID, config.motorType, config.controllerType);
            SparkBaseConfig followerConfig = null;
            switch (config.controllerType) {
                case CANSparkMax:
                    followerConfig = new SparkMaxConfig();
                    break;
                case CANSparkFlex:
                    followerConfig = new SparkFlexConfig();
                default:
                    break;
            }
           
            followerConfig.follow(main.motor, config.followers[i].inverted);

            this.followers[i].motor.configure(
                followerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
            );
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
