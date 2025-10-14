package frc.robot.subsystems.ExampleIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.MotorComponent;
import frc.lib.component.ServoMotorComponent;
import frc.lib.hardware.motor.SparkBaseIO;

public class ExampleIntake extends ComponentSubsystem {
    private final ServoMotorComponent<SparkBaseIO> pivot;
    private final MotorComponent<SparkBaseIO> rollers;

    public ExampleIntake() {
        pivot = registerComponent("Pivot", PivotConstants.getPivot());
        rollers = registerComponent("Rollers", RollerConstants.getRoller());
    }

    public Command intake() {
        return sequence(
            parallel(
                pivot.applyPositionSetpointCommandWithWait(PivotConstants.deploySetpoint),
                rollers.applySetpointCommand(RollerConstants.idleSetpoint)
            ),
            rollers.applySetpointCommand(RollerConstants.inwardsSetpoint)
        );
    }

    public Command spit() {
        return parallel(
            pivot.applySetpointCommand(PivotConstants.unjamSetpoint),
            rollers.applySetpointCommand(RollerConstants.spitSetpoint),
            Commands.waitSeconds(0.1) // TODO REMOVE (temporary so it shows up on commandscheduler log for debugging)
        );
    }

    public Command stow() {
        return parallel(
            pivot.applySetpointCommand(PivotConstants.stowSetpoint),
            rollers.applySetpointCommand(RollerConstants.idleSetpoint),
            Commands.waitSeconds(0.1) // TODO REMOVE (temporary so it shows up on commandscheduler log for debugging)
        );
    }
}
