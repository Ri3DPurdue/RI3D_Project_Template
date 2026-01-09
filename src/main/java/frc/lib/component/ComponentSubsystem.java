package frc.lib.component;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

/**
 * A subsystem designed to use sub-components.
 * 
 * <p>
 * ComponentSubsystem provides a convenient base class for subsystems that
 * aggregate multiple Component instances. It handles automatic periodic
 * updates, logging, and command creation with proper subsystem requirements.
 * 
 * <p>
 * Components are registered by name using
 * {@link #registerComponent(String, Component)} and are automatically updated
 * during the periodic cycle. The subsystem also automates logging of all
 * registered components.
 * 
 * @see Component
 * @see Loggable
 */
public class ComponentSubsystem extends SubsystemBase implements Loggable {
    /** Map to store components contained within the subsystem */
    private Map<String, Component> namedComponents = new HashMap<>();

    /**
     * Registers a component with this subsystem.
     * 
     * <p>
     * The component is stored by name and will be automatically updated during
     * periodic cycles and included in logging operations.
     * 
     * @param <C>       The component type
     * @param name      The unique name for this component
     * @param component The component instance to register
     * @return The same component instance for convenient chaining
     */
    protected <C extends Component> C registerComponent(String name, C component) {
        namedComponents.put(name, component);
        return component;
    }

    /**
     * Performs periodic updates for all registered components.
     * 
     * <p>
     * This method is automatically called by the subsystem scheduler and iterates
     * through all registered components, calling their periodic methods in
     * sequence.
     * This ensures that all components are updated each control loop.
     */
    @Override
    public void periodic() {
        for (Component componentWithName : namedComponents.values()) {
            componentWithName.periodic();
        }
    }

    /**
     * Logs all registered components.
     * 
     * <p>
     * This method iterates through all registered components and logs their state
     * using the Logger utility. The subsystem name is prepended to the component
     * names
     * in the log output for organization.
     * 
     * @param name The name prefix for logging this subsystem
     */
    @Override
    public void log(String name) {
        for (Map.Entry<String, Component> namedComponent : namedComponents.entrySet()) {
            Logger.log(name, namedComponent.getKey(), namedComponent.getValue());
        }
    }

    /**
     * Adds this subsystem to the requirement list of the passed in command
     * @param command The command that requires this subsystem, which will be modified
     * @return The passed in command, for call chaining
     */
    protected Command withRequirement(Command command) {
        command.addRequirements(this); 
        return command;
    }
}
