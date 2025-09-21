package frc.lib.hardware.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.util.Units;
import frc.lib.io.motor.MotorOutputs;
import frc.lib.sim.SimObject;

public class TalonFXIOSim extends TalonFXIO {
    private SimObject sim;
    public TalonFXIOSim(CANBus canbus, int leaderID, int[] followerIds, boolean[] followerInversion, SimObject simObject) {
        super(canbus, leaderID, followerIds, followerInversion);
    }

    @Override
    protected void updateOutputs(MotorOutputs[] outputs) {
        sim.setVoltage(motors[0].getMotorVoltage().getValueAsDouble());
        sim.update();
        for (TalonFX motor : motors) {
            TalonFXSimState simState = motor.getSimState();
            simState.setRawRotorPosition(Units.rotationsToRadians(sim.getPosition()));
            simState.setRawRotorPosition(Units.rotationsToRadians(sim.getVelocity()));
        }
        super.updateOutputs(outputs);
    }
}
