package frc.lib.component;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.Setpoint;

public class FlywheelMotorComponent<M extends MotorIO> extends MotorComponent<M> {
    protected double epsilonThreshold;

    public FlywheelMotorComponent(M motorIO, double epsilon) {
        super(motorIO);
        epsilonThreshold = epsilon;
    }

    public boolean nearVelocity(double velocity) {
        return MathUtil.isNear(velocity, getVelocity(), epsilonThreshold);
    }

    /**
     * Get a command that waits until a given velocity is reached as defined by the epsilon threshold.
     * 
     * @param velocity Mechanism velocity to wait until reached in radians or meters.
     * @return A command that waits until a given velocity is reached.
     */
    public Command waitForVelocityCommand(double velocity) {
        return Commands.waitUntil(() -> nearVelocity(velocity));
    }

    /**
     * Get a command which applies a velocity setpoint and then waits until the target velocity applied is reached. Only to be used with velocity control setpoints.
     * 
     * @param setpoint Velocity setpoint to target.
     * @return A command which applies a posiiton setpoint and waits for it to be reached. 
     */
    public Command applyVelocitySetpointCommandWithWait(Setpoint setpoint) {
        if (!setpoint.outputType.isVelocityControl()) {
            throw new IllegalArgumentException("applyVelocitySetpointCommandWithWait requires a velocity setpoint");
        }
        return waitForVelocityCommand(setpoint.value).deadlineFor(applySetpointCommand(setpoint));
    }
    
}
