package frc.robot.subsystems.exampleIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.FlywheelMotorComponent;
import frc.lib.component.MotorComponent;
import frc.lib.component.ServoMotorComponent;
import frc.lib.io.motor.rev.SparkBaseIO;

public class ExampleIntake extends ComponentSubsystem {
    private final ServoMotorComponent<SparkBaseIO> pivot;
    private final MotorComponent<SparkBaseIO> rollers;
    private final FlywheelMotorComponent<SparkBaseIO> indexer;

    public ExampleIntake() {
        pivot = registerComponent("Pivot", PivotConstants.getPivot());
        rollers = registerComponent("Rollers", RollerConstants.getRoller());
        indexer = registerComponent("Indexer", IndexerConstants.getIndexer());
    }

    public Command idleRollers() {
        return withRequirement(
            Commands.parallel(
                rollers.applySetpointCommand(RollerConstants.idleSetpoint),
                indexer.applySetpointCommand(IndexerConstants.idleSetpoint)
            )
        );
    }

    public Command intake() {
        return withRequirement(
            Commands.sequence(
                Commands.parallel(
                    pivot.applyPositionSetpointCommandWithWait(PivotConstants.deploySetpoint),
                    idleRollers()
                ),
                rollers.applySetpointCommand(RollerConstants.inwardsSetpoint),
                indexer.applySetpointCommand(IndexerConstants.feedSetpoint)
            ))
        ;
    }

    public Command spit() {
        return withRequirement(
            Commands.parallel(
                pivot.applySetpointCommand(PivotConstants.unjamSetpoint),
                rollers.applySetpointCommand(RollerConstants.spitSetpoint),
                indexer.applySetpointCommand(IndexerConstants.spitSetpoint)
            )
        );
    }

    public Command stow() {
        return withRequirement(
            Commands.parallel(
                pivot.applySetpointCommand(PivotConstants.stowSetpoint),
                idleRollers()
            )
        );
    }
}
