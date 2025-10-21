package frc.lib.component;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Util.UnitsUtil;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.setpoints.VelocitySetpoint;

public class FlywheelMotorComponent<M extends MotorIO> extends MotorComponent<M> {
    protected AngularVelocity epsilonThreshold;

    public FlywheelMotorComponent(M motorIO, AngularVelocity epsilon) {
        super(motorIO);
        epsilonThreshold = epsilon;
    }

    public boolean nearVelocity(AngularVelocity velocity) {
        return UnitsUtil.isNear(velocity, getVelocity(), epsilonThreshold);
    }

    /**
     * Get a command that waits until a given velocity is reached as defined by the epsilon threshold.
     * 
     * @param velocity Mechanism velocity to wait until reached in radians or meters.
     * @return A command that waits until a given velocity is reached.
     */
    public Command waitForVelocityCommand(AngularVelocity velocity) {
        return Commands.waitUntil(() -> nearVelocity(velocity));
    }

    /**
     * Get a command which applies a velocity setpoint and then waits until the target velocity applied is reached. Only to be used with velocity control setpoints.
     * 
     * @param setpoint Velocity setpoint to target.
     * @return A command which applies a posiiton setpoint and waits for it to be reached. 
     */
    public Command applyVelocitySetpointCommandWithWait(VelocitySetpoint setpoint) {
        return waitForVelocityCommand(setpoint.get()).deadlineFor(applySetpointCommand(setpoint));
    }
    
}
