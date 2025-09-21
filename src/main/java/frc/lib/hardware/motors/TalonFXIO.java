package frc.lib.hardware.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorOutputs;

public class TalonFXIO extends MotorIO {
    private TalonFX[] motors;
    private PositionVoltage positionRequest;
    private VelocityVoltage velocityRequest;
    private MotionMagicVoltage motionMagicRequest;

    public TalonFXIO(CANBus canbus, int leaderID, int... followerIds) {
        super(followerIds.length);
        motors = new TalonFX[followerIds.length + 1];
        motors[0] = new TalonFX(leaderID, canbus);
        for (int i = 1; i < followerIds.length + 1; i++) {
            motors[i] = new TalonFX(followerIds[i], canbus);
        }
    }

    @Override
    protected void updateOutputs(MotorOutputs[] outputs) {
        for (int i = 0; i < outputs.length; i++) {
            outputs[i] = new MotorOutputs(
                motors[i].getPosition().getValueAsDouble(), 
                motors[i].getVelocity().getValueAsDouble(), 
                motors[i].getMotorVoltage().getValueAsDouble(), 
                motors[i].getStatorCurrent().getValueAsDouble(), 
                motors[i].getSupplyCurrent().getValueAsDouble(), 
                motors[i].getDeviceTemp().getValueAsDouble()
            );
        }
    }

    @Override
    protected void setVoltage(double voltage) {
        motors[0].setVoltage(voltage);
    }

    @Override
    protected void setCurrent(double current) {
        throw new UnsupportedOperationException("TalonFX does not support current control without Phoenix Pro");
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
        motors[0].setControl(motionMagicRequest.withPosition(position));
    }

    @Override
    protected void setPercentage(double percentage) {
        motors[0].set(percentage);
    }

    @Override
    protected void setIdle() {
        motors[0].setVoltage(0);
    }
}
