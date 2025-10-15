package frc.robot.subsystems.ExampleIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.FlywheelMotorComponent;
import frc.lib.component.MotorComponent;
import frc.lib.component.ServoMotorComponent;
import frc.lib.hardware.motor.SparkBaseIO;

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
        return parallel(
            rollers.applySetpointCommand(RollerConstants.idleSetpoint),
            indexer.applySetpointCommand(IndexerConstants.idleSetpoint)
        );
    }

    public Command intake() {
        return sequence(
            parallel(
                pivot.applyPositionSetpointCommandWithWait(PivotConstants.deploySetpoint),
                idleRollers()
            ),
            rollers.applySetpointCommand(RollerConstants.inwardsSetpoint),
            indexer.applySetpointCommand(IndexerConstants.feedSetpoint)
        );
    }

    public Command spit() {
        return parallel(
            pivot.applySetpointCommand(PivotConstants.unjamSetpoint),
            rollers.applySetpointCommand(RollerConstants.spitSetpoint),
            indexer.applySetpointCommand(IndexerConstants.spitSetpoint)
        );
    }

    public Command stow() {
        return parallel(
            pivot.applySetpointCommand(PivotConstants.stowSetpoint),
            idleRollers()
        );
    }
}
