package frc.lib.component;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Util.logging.Loggable;
import frc.lib.Util.logging.Logger;

/**
 * A subsystem that manages a collection of named components.
 * 
 * <p>
 * ComponentSubsystem provides a convenient base class for subsystems that
 * aggregate multiple Component instances. It handles automatic periodic
 * updates, logging, and command creation with proper subsystem requirements.
 * 
 * <p>
 * Components are registered by name using
 * {@link #registerComponent(String, Component)} and are automatically updated
 * during the periodic cycle. The subsystem also manages logging of all
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
     * Creates a sequential command that requires this subsystem.
     * 
     * <p>
     * This is a convenience method that creates a sequence of commands and
     * automatically
     * adds this subsystem as a requirement. Commands execute in order, with each
     * command
     * completing before the next begins.
     * 
     * @param commands The commands to execute in sequence
     * @return A new sequential command with this subsystem as a requirement
     */
    protected Command sequence(Command... commands) {
        Command cmd = Commands.sequence(commands);
        cmd.addRequirements(this);
        return cmd;
    }

    /**
     * Creates a parallel command that requires this subsystem.
     * 
     * <p>
     * This is a convenience method that creates a group of parallel commands and
     * automatically adds this subsystem as a requirement. Commands execute
     * simultaneously
     * and the group completes when all commands finish.
     * 
     * @param commands The commands to execute in parallel
     * @return A new parallel command with this subsystem as a requirement
     */
    protected Command parallel(Command... commands) {
        Command cmd = Commands.parallel(commands);
        cmd.addRequirements(this);
        return cmd;
    }

    /**
     * Wraps a command to require this subsystem.
     * 
     * <p>
     * This is a convenience method that automatically adds this subsystem as a
     * requirement for the provided command. Needed for ensuring that commands have
     * the proper subsystem requirements set before scheduling.
     * 
     * @param command The command to wrap
     * @return The same command with this subsystem added as a requirement
     */
    protected Command command(Command command) {
        command.addRequirements(this);
        return command;
    }
}
