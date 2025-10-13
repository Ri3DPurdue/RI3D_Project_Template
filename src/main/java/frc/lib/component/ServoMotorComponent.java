package frc.lib.component;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Util.UnitsUtil;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.Setpoint;

public class ServoMotorComponent<M extends MotorIO> extends MotorComponent<M> {
    protected Angle epsilonThreshold;

    public ServoMotorComponent(M motorIO, Angle epsilon) {
        super(motorIO);
        epsilonThreshold = epsilon;
    }

    public boolean nearPosition(Angle position) {
        return UnitsUtil.isNear(position, getPosition(), epsilonThreshold);
    }

    /**
     * Get a command that waits until a given position is reached as defined by the epsilon threshold.
     * 
     * @param position Mechanism position to wait until reached in radians or meters.
     * @return A command that waits until a given position is reached.
     */
    public Command waitForPositionCommand(Angle position) {
        return Commands.waitUntil(() -> nearPosition(position));
    }

    /**
     * Get a command which applies a position setpoint and then waits until the target position applied is reached. Only to be used with position control setpoints.
     * 
     * @param setpoint Position setpoint to target.
     * @return A command which applies a posiiton setpoint and waits for it to be reached. 
     */
    public Command applyPositionSetpointCommandWithWait(Setpoint setpoint) {
        if (!setpoint.outputType.isPositionControl()) {
            throw new IllegalArgumentException("applyPositionSetpointCommandWithWait requires a position setpoint");
        }
        return waitForPositionCommand(BaseUnits.AngleUnit.of(setpoint.value)).deadlineFor(applySetpointCommand(setpoint));
    }
    
}
