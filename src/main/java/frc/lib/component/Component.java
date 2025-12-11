package frc.lib.component;

import frc.lib.Util.logging.Loggable;

/**
 * Base interface for all components in the robot system.
 * 
 * <p>Components are modular units that encapsulate hardware or logical functionality.
 * They handle their own periodic updates and logging, making them composable building
 * blocks for subsystems.
 * 
 * <p>All components must implement periodic updates and logging functionality through
 * this interface.
 * 
 * @see ComponentSubsystem
 * @see Loggable
 */
public interface Component extends Loggable {
    /**
     * Performs periodic updates for this component.
     * 
     * <p>
     * This method is called regularly by the component's parent subsystem or
     * scheduler to update the component's state, process sensor readings, or
     * perform other recurring tasks.
     * <p>
     * Should be identical to WPILib's periodic method.
     */
    public void periodic();
}
