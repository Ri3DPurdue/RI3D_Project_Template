package frc.lib.io.motor.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.Pair;
import frc.lib.mechanismSim.SimObject;

/**
 * A class that represents a simulated {@link TalonFX}
 */
public class TalonFXIOSim extends TalonFXIO {
    private final SimObject sim;
    private final double gearing;

    /**
     * Constructs a {@link TalonFXIOSim}
     * @param leaderID The can ID of the leader motor
     * @param canbus The canbus the motor's and its followers are on
     * @param config The {@link TalonFXConfiguration} to apply to the leader motor
     * @param simObject The object which will simulate the physics for this group of motors
     * @param followers An array of integer boolean pairs which represent the can ID and inversion relative to the main motor for each follower
     */
    @SuppressWarnings("unchecked")
    public TalonFXIOSim(int leaderID, String canbus, TalonFXConfiguration config, SimObject simObject, double gearing, Pair<Integer, Boolean>... followers ) {
        super(leaderID, canbus, config, followers);
        sim = simObject;
        this.gearing = gearing;
        updateMotorSimState();
    }

    @Override
    public void periodic() {
        sim.setVoltage(motors[0].getMotorVoltage().getValue());
        sim.update();
        updateMotorSimState();
        super.periodic();
    }

    /**
     * Updates the internal sim states for all of the talon's in the group
     */
    private void updateMotorSimState() {
        for (TalonFX motor : motors) {
            TalonFXSimState simState = motor.getSimState();
            simState.setRawRotorPosition(sim.getPosition().times(gearing));
            simState.setRotorVelocity(sim.getVelocity().times(gearing));
        }
    }
}
