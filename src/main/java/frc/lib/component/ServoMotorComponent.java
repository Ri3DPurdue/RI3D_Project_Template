package frc.lib.component;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.Setpoint;
import frc.lib.util.Util;

public class ServoMotorComponent<M extends MotorIO> extends MotorComponent<M> {
    protected double epsilonThreshold;

    public ServoMotorComponent(M motorIO, double epsilon) {
        super(motorIO);
        epsilonThreshold = epsilon;
    }

    public boolean nearPosition(double position) {
        return Util.epsilonEquals(getPosition(), position, epsilonThreshold);
    }

    /**
     * Get a command that waits until a given position is reached as defined by the epsilon threshold.
     * 
     * @param position Mechanism position to wait until reached in radians or meters.
     * @return A command that waits until a given position is reached.
     */
    public Command waitForPositionCommand(double position) {
        return Commands.waitUntil(() -> nearPosition(position));
    }

    /**
     * Get a command which applies a position setpoint and then waits until the target position applied is reached. Only to be used with position control setpoints.
     * 
     * @param setpoint Position setpoint to target.
     * @return A command which applies a posiiton setpoint and waits for it to be reached. 
     */
    public Command applyPositionSetpointCommandWithWait(Setpoint setpoint) {
        return waitForPositionCommand(setpoint.value).deadlineFor(applySetpointCommand(setpoint));
    }
    
}
