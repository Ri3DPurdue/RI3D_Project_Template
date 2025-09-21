package frc.lib.hardware.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;

import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorOutputs;

/**
 * A class that represents a {@link TalonFXS}
 */
public class TalonFXSIO extends MotorIO {
    protected final TalonFXS[] motors;
    private PositionVoltage positionRequest;
    private VelocityVoltage velocityRequest;
    private MotionMagicVoltage profiledPositionRequest;
    private NeutralOut idleRequest;

    /**
     * Constructs a {@link TalonFXSIO}
     * @param canbus The canbus the motor's and its followers are on
     * @param leaderID The can ID of the leader motor
     * @param followerIds The canID of the follower motors
     * @param followerInversion Whether each follower is inverted with respect to the leader or not
     */
    public TalonFXSIO(CANBus canbus, int leaderID, int[] followerIds, boolean[] followerInversion) {
        super(followerIds.length);
        if (followerIds.length != followerInversion.length) {
            throw new IllegalArgumentException("Number of followers and followerInversion length must be equal");
        }
        motors = new TalonFXS[followerIds.length + 1];
        motors[0] = new TalonFXS(leaderID, canbus);
        for (int i = 1; i < followerIds.length + 1; i++) {
            motors[i] = new TalonFXS(followerIds[i], canbus);
            motors[i].setControl(new Follower(leaderID, followerInversion[i]));
        }
        positionRequest = new PositionVoltage(0);
        velocityRequest = new VelocityVoltage(0);
        profiledPositionRequest = new MotionMagicVoltage(0);
        idleRequest = new NeutralOut();
    }

    @Override
    protected void updateOutputs(MotorOutputs[] outputs) {
        for (int i = 0; i < outputs.length; i++) {
                outputs[i].position = motors[i].getPosition().getValueAsDouble();
                outputs[i].velocity = motors[i].getVelocity().getValueAsDouble();
                outputs[i].voltage = motors[i].getMotorVoltage().getValueAsDouble();
                outputs[i].statorCurrent = motors[i].getStatorCurrent().getValueAsDouble();
                outputs[i].supplyCurrent = motors[i].getSupplyCurrent().getValueAsDouble();
                outputs[i].temperatureCelsius = motors[i].getDeviceTemp().getValueAsDouble();
        }
    }

    @Override
    protected void setVoltage(double voltage) {
        motors[0].setVoltage(voltage);
    }

    @Override
    protected void setCurrent(double current) {
        throw new UnsupportedOperationException("TalonFXS does not support current control without Phoenix Pro");
    }

    @Override
    protected void setPosition(double position) {
        motors[0].setControl(positionRequest.withPosition(position));
    }

    @Override
    protected void setVelocity(double velocity) {
        motors[0].setControl(velocityRequest.withVelocity(velocity));
    }

    @Override
    protected void setProfiledPosition(double position) {
        motors[0].setControl(profiledPositionRequest.withPosition(position));
    }

    @Override
    protected void setPercentage(double percentage) {
        motors[0].set(percentage);
    }

    @Override
    protected void setIdle() {
        motors[0].setControl(idleRequest);
    }
}
