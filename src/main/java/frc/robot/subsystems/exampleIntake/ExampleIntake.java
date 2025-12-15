package frc.robot.subsystems.exampleIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.MotorComponent;
import frc.lib.component.ServoMotorComponent;
import frc.lib.io.motor.rev.SparkBaseIO;

public class ExampleIntake extends ComponentSubsystem {
    private final ServoMotorComponent<SparkBaseIO> pivot;
    private final MotorComponent<SparkBaseIO> rollers;

    public ExampleIntake() {
        pivot = registerComponent("Pivot", PivotConstants.getComponent());
        rollers = registerComponent("Rollers", RollerConstants.getComponent());
    }

    public Command intake() {
        return withRequirement(
            Commands.sequence(
                pivot.applyPositionSetpointCommandWithWait(PivotConstants.deploySetpoint),
                rollers.applySetpointCommand(RollerConstants.inwardsSetpoint)
            )
        );
    }

    public Command stow() {
        return withRequirement(
            Commands.parallel(
                pivot.applySetpointCommand(PivotConstants.stowSetpoint),
                rollers.applySetpointCommand(RollerConstants.idleSetpoint)
            )
        );
    }

    public Command unjam() {
        return withRequirement(
            Commands.parallel(
                pivot.applySetpointCommand(PivotConstants.unjamSetpoint),
                rollers.applySetpointCommand(RollerConstants.spitSetpoint)
            )
        );
    }
}
