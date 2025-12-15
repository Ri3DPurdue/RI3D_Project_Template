package frc.robot.subsystems.exampleClimber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.ServoMotorComponent;
import frc.lib.io.motor.rev.SparkBaseIO;

public class ExampleClimber extends ComponentSubsystem {
    private final ServoMotorComponent<SparkBaseIO> climber;

    public ExampleClimber() {
        climber = registerComponent("Climber", ClimberConstants.getComponent());
    }

    public Command extend() {
        return withRequirement(
            climber.applyPositionSetpointCommandWithWait(ClimberConstants.extendSetpoint)
        );
    }

    public Command pull() {
        return withRequirement(
            climber.applyPositionSetpointCommandWithWait(ClimberConstants.pullSetpoint)
        );
    }

    public Command stow() {
        return withRequirement(
            climber.applyPositionSetpointCommandWithWait(ClimberConstants.stowSetpoint)
        );
    }
}
