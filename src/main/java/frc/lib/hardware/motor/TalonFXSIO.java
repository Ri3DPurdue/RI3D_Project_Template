package frc.lib.hardware.motor;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.util.Units;
import frc.lib.io.motor.FollowerConfig;
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
     * @param leaderID The can ID of the leader motor
     * @param canbus The canbus the motor's and its followers are on
     * @param config The {@link TalonFXSConfiguration} to apply to the leader motor
     * @param followers An array of {@link FollowerConfig}s which configure the followers of this motor
     */
    public TalonFXSIO(int leaderID, CANBus canbus, TalonFXSConfiguration config, FollowerConfig... followers) {
        super(followers.length);
        motors = new TalonFXS[followers.length + 1];
        motors[0] = new TalonFXS(leaderID, canbus);
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
            status = motors[0].getConfigurator().apply(config);
        }
        for (int i = 1; i <= followers.length; i++) {
            motors[i] = new TalonFXS(followers[i].canID, canbus);
            motors[i].setControl(new Follower(leaderID, followers[i].inverted));
        }
        positionRequest = new PositionVoltage(0);
        velocityRequest = new VelocityVoltage(0);
        profiledPositionRequest = new MotionMagicVoltage(0);
        idleRequest = new NeutralOut();
    }

    @Override
    protected void updateOutputs(MotorOutputs[] outputs) {
        for (int i = 0; i < outputs.length; i++) {
                outputs[i].position = Units.rotationsToRadians(motors[i].getPosition().getValueAsDouble());
                outputs[i].velocity = Units.rotationsToRadians(motors[i].getVelocity().getValueAsDouble());
                outputs[i].statorVoltage = motors[i].getMotorVoltage().getValueAsDouble();
                outputs[i].supplyVoltage = motors[i].getSupplyVoltage().getValueAsDouble();
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
        motors[0].setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }

    @Override
    protected void setVelocity(double velocity) {
        motors[0].setControl(velocityRequest.withVelocity(Units.radiansToRotations(velocity)));
    }

    @Override
    protected void setProfiledPosition(double position) {
        motors[0].setControl(profiledPositionRequest.withPosition(Units.radiansToRotations(position)));
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
