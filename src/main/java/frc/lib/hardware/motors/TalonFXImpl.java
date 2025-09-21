package frc.lib.hardware.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorOutputs;

public class TalonFXImpl extends MotorIO {
    private TalonFX[] motors;

    public TalonFXImpl(CANBus canbus, int... ids) {
        super(ids.length);
        motors = new TalonFX[ids.length];
        for (int i = 0; i < ids.length; i++) {
            motors[i] = new TalonFX(ids[i], canbus);
        }
    }

    @Override
    protected void updateOutputs(MotorOutputs[] outputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateOutputs'");
    }

    @Override
    protected void setVoltage(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    @Override
    protected void setCurrent(double current) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCurrent'");
    }

    @Override
    protected void setPosition(double position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }

    @Override
    protected void setVelocity(double velocity) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVelocity'");
    }

    @Override
    protected void setProfiledPosition(double position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setProfiledPosition'");
    }

    @Override
    protected void setPercentage(double percentage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPercentage'");
    }

    @Override
    protected void setIdle() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setIdle'");
    }
}
