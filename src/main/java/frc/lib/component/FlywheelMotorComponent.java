package frc.lib.component;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.setpoints.VelocitySetpoint;
import frc.lib.util.UnitsUtil;

/**
 * A motor component specialized for velocity-controlled (usually flywheel) mechanisms.
 * 
 * <p>
 * This component extends {@link MotorComponent} to provide velocity-specific functionality,
 * including velocity tolerance checking and commands that wait for target velocities to be reached.
 * It is designed for mechanisms that need precise velocity control, such as shooters, intakes,
 * or flywheels.
 * 
 * <p>The component uses an epsilon (max error) threshold to determine when a target velocity has been
 * reached, accounting for real-world control variations and allowing for spin-up detection.
 * 
 * @param <M> the type of MotorIO implementation used by this component
 * @see MotorComponent
 * @see ServoMotorComponent
 */
public class FlywheelMotorComponent<M extends MotorIO> extends MotorComponent<M> {
    /** The velocity tolerance for determining if a target has been reached */
    protected AngularVelocity epsilonThreshold;

    /**
     * Constructs a new FlywheelMotorComponent with specified velocity tolerance.
     * 
     * @param motorIO the motor I/O implementation to control
     * @param epsilon the velocity tolerance threshold for target detection
     */
    public FlywheelMotorComponent(M motorIO, AngularVelocity epsilon) {
        super(motorIO);
        epsilonThreshold = epsilon;
    }

    /**
     * Checks if the current velocity is within tolerance of the target velocity.
     * 
     * <p>Uses the epsilon threshold to determine if the mechanism has reached
     * the target velocity within acceptable bounds.
     * 
     * @param velocity the target velocity to check against
     * @return true if the current velocity is within epsilon of the target, false otherwise
     */
    public boolean nearVelocity(AngularVelocity velocity) {
        return UnitsUtil.isNear(velocity, getVelocity(), epsilonThreshold);
    }

    /**
     * Creates a command that waits until the mechanism reaches a target velocity.
     * 
     * <p>The command finishes when the mechanism is within the epsilon threshold of
     * the target velocity. This is useful for ensuring a flywheel has spun up before
     * feeding game pieces or for sequencing velocity-dependent operations.
     * 
     * @param velocity the target velocity to wait for
     * @return a command that finishes when the velocity is reached
     */
    public Command waitForVelocityCommand(AngularVelocity velocity) {
        return Commands.waitUntil(() -> nearVelocity(velocity));
    }

    /**
     * Creates a command that applies a velocity setpoint and waits until it is reached.
     * 
     * <p>This command combines applying a velocity setpoint with waiting for the mechanism
     * to reach that velocity. The command finishes when the velocity is reached within
     * the epsilon threshold. Only use with velocity control setpoints.
     * 
     * @param setpoint the velocity setpoint to apply and wait for
     * @return a command that applies the setpoint and waits for completion
     */
    public Command applyVelocitySetpointCommandWithWait(VelocitySetpoint setpoint) {
        return waitForVelocityCommand(setpoint.get()).deadlineFor(applySetpointCommand(setpoint));
    }
    
}
