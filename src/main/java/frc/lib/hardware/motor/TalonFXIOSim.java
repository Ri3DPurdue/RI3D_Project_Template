package frc.lib.hardware.motor;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.Pair;
import frc.lib.sim.SimObject;

/**
 * A class that represents a simulated {@link TalonFX}
 */
public class TalonFXIOSim extends TalonFXIO {
    private SimObject sim;

    /**
     * Constructs a {@link TalonFXIOSim}
     * @param leaderID The can ID of the leader motor
     * @param canbus The canbus the motor's and its followers are on
     * @param config The {@link TalonFXConfiguration} to apply to the leader motor
     * @param simObject The object which will simulate the physics for this group of motors
     * @param followers An array of integer boolean pairs which represent the can ID and inversion relative to the main motor for each follower
     */
    @SuppressWarnings("unchecked")
    public TalonFXIOSim(int leaderID, CANBus canbus, TalonFXConfiguration config, SimObject simObject, Pair<Integer, Boolean>... followers ) {
        super(leaderID, canbus, config, followers);
    }

    @Override
    public void periodic() {
        sim.setVoltage(motors[0].getMotorVoltage().getValue());
        sim.update();
        for (TalonFX motor : motors) {
            TalonFXSimState simState = motor.getSimState();
            simState.setRawRotorPosition(sim.getPosition().times(config.Feedback.SensorToMechanismRatio));
            simState.setRotorVelocity(sim.getVelocity().times(config.Feedback.SensorToMechanismRatio));
        }
        super.periodic();
    }
}
