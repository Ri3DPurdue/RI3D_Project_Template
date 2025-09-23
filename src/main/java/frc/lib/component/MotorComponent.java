package frc.lib.component;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorOutputs;
import frc.lib.io.motor.Setpoint;

public class MotorComponent<M extends MotorIO> implements Component{
    private M io;

    @Override
    public void periodic() {
        io.periodic();
    }

    public void enable() {
        io.enable();
    }

    public void disable() {
        io.disable();
    }

    public boolean isEnabled() {
        return io.isEnabled();
    }

    /**
     * Gets the specific MotorIO for this component. Should only be used to access property specific to motor controller and not available through generic MotorIO.
     * 
     * @return
     */
    public M getMotorIO() {
        return io;
    }
    
    public MotorOutputs[] getMotorOutputs() {
        return io.getOutputs();
    }

    /**
     * Get the position of the main motor.
     * 
     * @return 
     */
    public double getPosition() {
        return getMotorOutputs()[0].position();
    }

    /**
     * Get the velocity of the main motor.
     * 
     * @return 
     */
    public double getVelocity() {
        return getMotorOutputs()[0].velocity();
    }

    /**
     * Get the voltage of the main motor.
     * 
     * @return 
     */
    public double getVoltage() {
        return getMotorOutputs()[0].voltage();
    }

    /**
     * Get the stator current of the main motor.
     * 
     * @return 
     */
    public double getStatorCurrent() {
        return getMotorOutputs()[0].statorCurrent();
    }

    /**
     * Get the supply current of the main motor.
     * 
     * @return 
     */
    public double getSupplyCurrent() {
        return getMotorOutputs()[0].supplyCurrent();
    }

    /**
     * Get the temperature of the main motor.
     * 
     * @return 
     */
    public double getTemperature() {
        return getMotorOutputs()[0].temperatureCelsius();
    }

    /**
     * Get the setpoint of the MotorIO.
     * 
     * @return 
     */
    public Setpoint getSetpoint() {
        return io.getCurrentSetpoint();
    }

    public void applySetpoint(Setpoint setpoint) {
        io.applySetpoint(setpoint);
    }

    public Command applySetpointCommand(Setpoint setpoint) {
        return Commands.runOnce(() -> applySetpoint(setpoint));
    }

    public Command followSetpointCommand(Supplier<Setpoint> supplier) {
        return Commands.run(() -> applySetpoint(supplier.get()));
    }

    public Command enableCommand() {
        return Commands.runOnce(() -> enable());
    }

    public Command disableCommand() {
        return Commands.runOnce(() -> disable());
    }
}
