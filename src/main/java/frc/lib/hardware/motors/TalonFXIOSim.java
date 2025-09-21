package frc.lib.hardware.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.util.Units;
import frc.lib.io.motor.MotorOutputs;
import frc.lib.sim.SimObject;

/**
 * A class that represents a simulated {@link TalonFX}
 */
public class TalonFXIOSim extends TalonFXIO {
    private SimObject sim;

    /**
     * Constructs a {@link TalonFXIOSim}
     * @param canbus The canbus the motor's and its followers are on
     * @param leaderID The can ID of the leader motor
     * @param followerIds The canID of the follower motors
     * @param followerInversion Whether each follower is inverted with respect to the leader or not
     * @param simObject The object which will simulate the physics for this group of motors
     */
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
            simState.setRotorVelocity(Units.rotationsToRadians(sim.getVelocity()));
        }
        super.updateOutputs(outputs);
    }
}
