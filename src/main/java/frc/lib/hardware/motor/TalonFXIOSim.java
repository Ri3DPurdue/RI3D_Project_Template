package frc.lib.hardware.motor;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.lib.io.motor.FollowerConfig;
import frc.lib.sim.SimObject;

/**
 * A class that represents a simulated {@link TalonFX}
 */
public class TalonFXIOSim extends TalonFXIO {
    private SimObject sim;
    private double conversionFactor;

    /**
     * Constructs a {@link TalonFXIOSim}
     * @param leaderID The can ID of the leader motor
     * @param canbus The canbus the motor's and its followers are on
     * @param config The {@link TalonFXConfiguration} to apply to the leader motor
     * @param simObject The object which will simulate the physics for this group of motors
     * @param conversionFactor This multiplied by mechanims units should yield rotations of the motor. So if you have a flywheel
     * with a five to one gear reduction, this would be 5/2pi, since if the flywheel rotates 1 radian, the motor rotated 5/2pi rotations
     * @param followers An array of {@link FollowerConfig}s which configure the followers of this motor
     */
    public TalonFXIOSim(int leaderID, CANBus canbus, TalonFXConfiguration config, SimObject simObject, double conversionFactor, FollowerConfig... followers ) {
        super(leaderID, canbus, config, followers);
        this.conversionFactor = conversionFactor;
    }

    @Override
    public void periodic() {
        sim.setVoltage(motors[0].getMotorVoltage().getValueAsDouble());
        sim.update();
        for (TalonFX motor : motors) {
            TalonFXSimState simState = motor.getSimState();
            simState.setRawRotorPosition(sim.getPosition() * conversionFactor);
            simState.setRotorVelocity(sim.getVelocity() * conversionFactor);
        }
        super.periodic();
    }
}
