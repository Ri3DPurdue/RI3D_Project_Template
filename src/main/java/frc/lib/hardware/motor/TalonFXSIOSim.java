package frc.lib.hardware.motor;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;

import frc.lib.io.motor.BaseConfig;
import frc.lib.sim.SimObject;

/**
 * A class that represents a simulated {@link TalonFXS}
 */
public class TalonFXSIOSim extends TalonFXSIO {
    private SimObject sim;
    private double conversionFactor;

    /**
     * Constructs a {@link TalonFXSIOSim}
     * @param canbus The canbus the motor's and its followers are on
     * @param leaderID The can ID of the leader motor
     * @param followerIds The canID of the follower motors
     * @param followerInversion Whether each follower is inverted with respect to the leader or not
     * @param simObject The object which will simulate the physics for this group of motors
     * @param conversionFactor This multiplied by mechanims units should yield rotations of the motor. So if you have a flywheel
     * with a five to one gear reduction, this would be 5/2pi, since if the flywheel rotates 1 radian, the motor rotated 5/2pi rotations
     */
    public TalonFXSIOSim(CANBus canbus, BaseConfig config, SimObject simObject) {
        super(canbus, config);
        this.conversionFactor = conversionFactor;
    }

    @Override
    public void periodic() {
        sim.setVoltage(motors[0].getMotorVoltage().getValueAsDouble());
        sim.update();
        for (TalonFXS motor : motors) {
            TalonFXSSimState simState = motor.getSimState();
            simState.setRawRotorPosition(sim.getPosition() * conversionFactor);
            simState.setRotorVelocity(sim.getVelocity() * conversionFactor);
        }
        super.periodic();
    }
}
