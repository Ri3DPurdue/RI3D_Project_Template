package frc.lib.io.motor.ctre;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorOutputs;
import frc.robot.Robot;

/**
 * A class that represents a {@link TalonFX}
 */
public class TalonFXIO extends MotorIO {
    protected final TalonFX[] motors;
    private PositionVoltage positionRequest;
    private VelocityVoltage velocityRequest;
    private MotionMagicVoltage profiledPositionRequest;
    private NeutralOut idleRequest;
    protected TalonFXConfiguration config;

    /**
     * Constructs a {@link TalonFXIO}
     * @param leaderID The can ID of the leader motor
     * @param canbus The canbus the motor's and its followers are on
     * @param config The {@link TalonFXConfiguration} to apply to the leader motor
     * @param followers An array of integer boolean pairs which represent the can ID and inversion relative to the main motor for each follower
     */
    @SuppressWarnings("unchecked")
    public TalonFXIO(int leaderID, String canbus, TalonFXConfiguration config, Pair<Integer, Boolean>... followers) {
        super(followers.length);
        motors = new TalonFX[followers.length + 1];
        motors[0] = new TalonFX(leaderID, canbus);
        for (int i = 1; i <= followers.length; i++) {
            motors[i] = new TalonFX(followers[i - 1].getFirst(), canbus);
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int j = 0; j < 5 && status != StatusCode.OK; j++) {
                status = motors[i].getConfigurator().apply(config);
            }
            motors[i].setControl(new Follower(leaderID, followers[i - 1].getSecond()));
        }
        reconfigure(config);
        positionRequest = new PositionVoltage(0);
        velocityRequest = new VelocityVoltage(0);
        profiledPositionRequest = new MotionMagicVoltage(0);
        idleRequest = new NeutralOut();
    }

    public void reconfigure(TalonFXConfiguration config) {
        this.config = config;
        Robot.blockingCalls.add(() -> {
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                status = motors[0].getConfigurator().apply(config);
            }
        });
        Robot.blockingCalls.add(() -> {
            for (int i = 1; i < motors.length; i++) {
                TalonFXConfiguration followerConfig = new TalonFXConfiguration();
                followerConfig.CurrentLimits = config.CurrentLimits;
                followerConfig.Feedback = config.Feedback;
                StatusCode status = StatusCode.StatusCodeNotInitialized;
                for (int j = 0; j < 5 && status != StatusCode.OK; j++) {
                    status = motors[j].getConfigurator().apply(config);
                }
            }
        });
    }

    @Override
    protected void updateOutputs(MotorOutputs[] outputs) {
        for (int i = 0; i < outputs.length; i++) {
            outputs[i].position = motors[i].getPosition().getValue();
            outputs[i].velocity = motors[i].getVelocity().getValue();
            outputs[i].statorVoltage = motors[i].getMotorVoltage().getValue();
            outputs[i].supplyVoltage = motors[i].getSupplyVoltage().getValue();
            outputs[i].statorCurrent = motors[i].getStatorCurrent().getValue();
            outputs[i].supplyCurrent = motors[i].getSupplyCurrent().getValue();
            outputs[i].temperature = motors[i].getDeviceTemp().getValue();
        }
    }

    @Override
    protected void setVoltage(Voltage voltage) {
        motors[0].setVoltage(voltage.in(Volts));
    }

    @Override
    protected void setCurrent(Current current) {
        throw new UnsupportedOperationException("TalonFX does not support current control without Phoenix Pro");
    }

    @Override
    protected void setPosition(Angle angle) {
        motors[0].setControl(positionRequest.withPosition(angle));
    }

    @Override
    protected void setVelocity(AngularVelocity angularVelocity) {
        motors[0].setControl(velocityRequest.withVelocity(angularVelocity));
    }

    @Override
    protected void setProfiledPosition(Angle angle) {
        motors[0].setControl(profiledPositionRequest.withPosition(angle));
    }

    @Override
    protected void setIdle() {
        motors[0].setControl(idleRequest);
    }

    @Override
    public void useSoftLimits(boolean use) {
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = use;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = use;
        reconfigure(config);
    }

    @Override
    public void resetPosition(Angle position) {
        for (TalonFX fx : motors) {
            fx.setPosition(position);
        }
    }
}
