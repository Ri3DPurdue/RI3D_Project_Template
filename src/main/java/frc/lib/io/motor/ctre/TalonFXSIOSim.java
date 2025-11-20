package frc.lib.io.motor.ctre;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSSimState;

import edu.wpi.first.math.Pair;
import frc.lib.mechanismSim.SimObject;

/**
 * A class that represents a simulated {@link TalonFXS}
 */
public class TalonFXSIOSim extends TalonFXSIO {
    private SimObject sim;

    /**
     * Constructs a {@link TalonFXSIOSim}
     * @param leaderID The can ID of the leader motor
     * @param canbus The canbus the motor's and its followers are on
     * @param config The {@link TalonFXSConfiguration} to apply to the leader motor
     * @param simObject The object which will simulate the physics for this group of motors
     * @param followers An array of integer boolean pairs which represent the can ID and inversion relative to the main motor for each follower
     */
    @SuppressWarnings("unchecked")
    public TalonFXSIOSim(int leaderID, CANBus canbus, TalonFXSConfiguration config, SimObject simObject, Pair<Integer, Boolean>... followers ) {
        super(leaderID, canbus, config, followers);
    }

    @Override
    public void periodic() {
        sim.setVoltage(motors[0].getMotorVoltage().getValue());
        sim.update();
        for (TalonFXS motor : motors) {
            TalonFXSSimState simState = motor.getSimState();
            simState.setRawRotorPosition(sim.getPosition().times(config.ExternalFeedback.SensorToMechanismRatio));
            simState.setRotorVelocity(sim.getVelocity().times(config.ExternalFeedback.SensorToMechanismRatio));
        }
        super.periodic();
    }
}
