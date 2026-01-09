package frc.lib.io.motor.ctre;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorOutputs;
import frc.robot.Robot;

/**
 * A class that represents a {@link TalonFXS}
 */
public class TalonFXSIO extends MotorIO {
    protected final TalonFXS[] motors;
    private PositionVoltage positionRequest;
    private VelocityVoltage velocityRequest;
    private MotionMagicVoltage profiledPositionRequest;
    private NeutralOut idleRequest;
    protected TalonFXSConfiguration config;

    /**
     * Constructs a {@link TalonFXSIO}
     * @param leaderID The can ID of the leader motor
     * @param canbus The canbus the motor's and its followers are on
     * @param config The {@link TalonFXSConfiguration} to apply to the leader motor
     * @param followers An array of integer boolean pairs which represent the can ID and inversion relative to the main motor for each follower
     */
    @SuppressWarnings("unchecked")
    public TalonFXSIO(int leaderID, String canbus, TalonFXSConfiguration config, Pair<Integer, Boolean>... followers) {
        super(followers.length);
        motors = new TalonFXS[followers.length + 1];
        motors[0] = new TalonFXS(leaderID, canbus);
        for (int i = 1; i <= followers.length; i++) {
            motors[i] = new TalonFXS(followers[i].getFirst(), canbus);
            motors[i].setControl(new Follower(leaderID, followers[i].getSecond()));
        }
        reconfigure(config);
        positionRequest = new PositionVoltage(0);
        velocityRequest = new VelocityVoltage(0);
        profiledPositionRequest = new MotionMagicVoltage(0);
        idleRequest = new NeutralOut();
    }

    /**
     * Applies the given config to the talon fx. Note that this is done asynchronously,
     * so code after this can't assume the config has been applied
     * @param config The config to apply
     */
    public void reconfigure(TalonFXSConfiguration config) {
        this.config = config;
        Robot.submitBlockingCall(() -> {
            for (TalonFXS fxs: motors) {
                StatusCode status = StatusCode.StatusCodeNotInitialized;
                for (int j = 0; j < 5 && status != StatusCode.OK; j++) {
                    status = fxs.getConfigurator().apply(config);
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
        throw new UnsupportedOperationException("TalonFXS does not support current control without Phoenix Pro");
    }

    @Override
    protected void setPosition(Angle position) {
        motors[0].setControl(positionRequest.withPosition(position));
    }

    @Override
    protected void setVelocity(AngularVelocity velocity) {
        motors[0].setControl(velocityRequest.withVelocity(velocity));
    }

    @Override
    protected void setProfiledPosition(Angle position) {
        motors[0].setControl(profiledPositionRequest.withPosition(position));
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
        Robot.submitBlockingCall(() -> {
            for (TalonFXS fxs: motors) {
                StatusCode status = StatusCode.StatusCodeNotInitialized;
                for (int j = 0; j < 5 && status != StatusCode.OK; j++) {
                    fxs.setPosition(position);
                }
            }
        });
    }
}
