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
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorOutputs;
import static com.revrobotics.spark.SparkBase.ControlType.*;

public class SparkBaseIO extends MotorIO {
    /**
     * Identifier enum for whether a motor controller is a spark base or spark max.
     */
    public static enum ControllerType {
        CANSparkMax,
        CANSparkFlex
    }
    /**
     * Inner class for exploding a generic Spark motor controller
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
        public final SparkBaseConfigAccessor accessor;

        public Exploded(
            int id, MotorType type, ControllerType sparkType
        ) {
            switch (sparkType) {
                case CANSparkFlex:
                    SparkFlex flex = new SparkFlex(id, type);
                    this.motor = flex;
                    this.accessor = flex.configAccessor;
                    break;
                case CANSparkMax:
                    SparkMax max = new SparkMax(id, type);
                    this.motor = max;
                    this.accessor = max.configAccessor;
                    
                    break;

                default:
                    motor = null;
                    accessor = null;
                    break;
            }
            
            this.controller = motor.getClosedLoopController();
            this.encoder = motor.getEncoder();
        }
    }

    protected final ControllerType type;
    protected final Exploded main;
    private final SparkBaseConfig config;
    protected final Exploded[] followers;

    private boolean forwardLimitEnabled;
    private boolean reverseLimitEnabled;

    /**
     * Creates a sparkBaseIO
     * @param type Whether the motor is brushed or brushless.
     * @see com.revrobotics.spark.SparkLowLevel.MotorType
     * @param sparkType Enum that indicates whether the motor controller is a spark max or a spark flex
     * @param mainMotor The id of the main motor
     * @param followers The id of any following motors, and whether they are inverted
     */
    @SuppressWarnings({ "unchecked" })
    public SparkBaseIO(
        MotorType type,
        SparkBaseConfig mainConfig,
        int mainMotor,
        Pair<Integer, Boolean>... followers
    ) {
        super(followers.length);

        if (mainConfig instanceof SparkMaxConfig) {
            this.type = ControllerType.CANSparkMax;
            config = new SparkMaxConfig();
        } else if (mainConfig instanceof SparkFlexConfig) {
            this.type = ControllerType.CANSparkFlex;
            config = new SparkFlexConfig();
        } else {
            throw new IllegalArgumentException("Main config must either be a spark max or spark flex config");
        }
        
        config.apply(mainConfig);
        
        main = new Exploded(mainMotor, type, this.type);
        main.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        forwardLimitEnabled = main.accessor.softLimit.getForwardSoftLimitEnabled();
        reverseLimitEnabled = main.accessor.softLimit.getReverseSoftLimitEnabled();

        this.followers = new Exploded[followers.length];

        for (int i = 0; i < followers.length; i++) {
            Pair<Integer, Boolean> follower = followers[i];
            this.followers[i] = new Exploded(follower.getFirst(), type, this.type);
            SparkBaseConfig config = null;
            switch (this.type) {
                case CANSparkMax:
                    config = new SparkMaxConfig();
                    break;
                case CANSparkFlex:
                    config = new SparkFlexConfig();
                default:
                    break;
            }
           
            config.follow(mainMotor, follower.getSecond());

            this.followers[i].motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    public void reconfigure(SparkBaseConfig config) {
        if (config == this.config) {
            // Internal for more easy reconfiguring
        } if (this.type == ControllerType.CANSparkMax && !(config instanceof SparkMaxConfig)) {
            throw new IllegalArgumentException("Must configure a spark max with a spark max config");
        } else if (this.type == ControllerType.CANSparkFlex && !(config instanceof SparkFlexConfig)) {
            throw new IllegalArgumentException("Must configure a spark flex with a spark flex config");
        } else {
            this.config.apply(config);
        }

        main.motor.configure(this.config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        forwardLimitEnabled = main.accessor.softLimit.getForwardSoftLimitEnabled();
        reverseLimitEnabled = main.accessor.softLimit.getReverseSoftLimitEnabled();
    }

    /**
     * helper method for loading the data from a motor into the outputs
     * @param controller
     * @param outputs
     */
    private static void loadOutputs(Exploded controller, MotorOutputs outputs) {
        double output = controller.motor.getAppliedOutput();

        outputs.statorCurrent = Units.Amps.of(controller.motor.getOutputCurrent());
        outputs.supplyCurrent = outputs.statorCurrent.times(output);

        outputs.supplyVoltage = Units.Volts.of(controller.motor.getBusVoltage());
        outputs.statorVoltage = outputs.supplyVoltage.times(output);

        outputs.position = Units.Rotations.of(controller.encoder.getPosition());
        outputs.velocity = Units.RPM.of(controller.encoder.getVelocity());
        
        outputs.temperatureCelsius = Units.Celsius.of(controller.motor.getMotorTemperature());
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

	@Override
	public void useSoftLimits(boolean use) {
        this.config.softLimit
            .forwardSoftLimitEnabled(use && forwardLimitEnabled)
            .reverseSoftLimitEnabled(use && reverseLimitEnabled);

        main.motor.configure(this.config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void resetPosition(Angle position) {
        main.encoder.setPosition(position.in(Units.Rotations));
	}
}
