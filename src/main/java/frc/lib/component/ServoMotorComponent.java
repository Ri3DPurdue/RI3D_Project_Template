package frc.lib.component;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.setpoints.PositionSetpoint;
import frc.lib.util.UnitsUtil;

/**
 * A motor component specialized for position-controlled servo mechanisms.
 * 
 * <p>This component extends {@link MotorComponent} to provide position-specific functionality,
 * including position tolerance checking and commands that wait for target positions to be reached.
 * It is designed for mechanisms that need precise position control, such as arms, elevators, or
 * rotating turrets.
 * 
 * <p>The component uses an epsilon threshold to determine when a target position has been
 * reached, accounting for real-world control imperfections.
 * 
 * @param <M> the type of MotorIO implementation used by this component
 * @see MotorComponent
 * @see HomingServoMotorComponent
 */
public class ServoMotorComponent<M extends MotorIO> extends MotorComponent<M> {
    /** The position tolerance for determining if a target has been reached */
    protected Angle epsilonThreshold;

    /**
     * Constructs a new ServoMotorComponent with specified tolerance and starting position.
     * 
     * @param motorIO the motor I/O implementation to control
     * @param epsilon the position tolerance threshold for target detection
     * @param startAngle the initial position to set on the encoder
     */
    public ServoMotorComponent(M motorIO, Angle epsilon, Angle startAngle) {
        super(motorIO);
        epsilonThreshold = epsilon;
        resetPosition(startAngle);
    }

    /**
     * Checks if the current position is within tolerance of the target position.
     * 
     * <p>Uses the epsilon threshold to determine if the mechanism has reached
     * the target position within acceptable bounds.
     * 
     * @param position the target position to check against
     * @return true if the current position is within epsilon of the target, false otherwise
     */
    public boolean nearPosition(Angle position) {
        return UnitsUtil.isNear(position, getPosition(), epsilonThreshold);
    }

    /**
     * Creates a command that waits until the mechanism reaches a target position.
     * 
     * <p>The command finishes when the mechanism is within the epsilon threshold of
     * the target position. This is useful for sequencing commands that depend on
     * the mechanism being at a specific position.
     * 
     * @param position the target position to wait for
     * @return a command that finishes when the position is reached
     */
    public Command waitForPositionCommand(Angle position) {
        return Commands.waitUntil(() -> nearPosition(position));
    }

    /**
     * Creates a command that applies a position setpoint and waits until it is reached.
     * 
     * <p>This command combines applying a position setpoint with waiting for the mechanism
     * to reach that position. The command finishes when the position is reached within
     * the epsilon threshold. Only use with position control setpoints.
     * 
     * @param setpoint the position setpoint to apply and wait for
     * @return a command that applies the setpoint and waits for completion
     */
    public Command applyPositionSetpointCommandWithWait(PositionSetpoint setpoint) {
        return waitForPositionCommand(setpoint.get()).deadlineFor(applySetpointCommand(setpoint));
    }
    
}
