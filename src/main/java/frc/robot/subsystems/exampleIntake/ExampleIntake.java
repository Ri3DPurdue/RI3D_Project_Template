package frc.robot.subsystems.exampleIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.DigitalIOComponent;
import frc.lib.component.HomingServoMotorComponent;
import frc.lib.component.MotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;
import frc.lib.io.motor.rev.SparkBaseIO;

public class ExampleIntake extends ComponentSubsystem {
    private final HomingServoMotorComponent<TalonFXIO> pivot;
    private final MotorComponent<SparkBaseIO> rollers;
    private final DigitalIOComponent beamBreak;

    public ExampleIntake() {
        pivot = registerComponent("Pivot", PivotConstants.getComponent());
        rollers = registerComponent("Rollers", RollerConstants.getComponent());
        beamBreak = registerComponent("Beam Break", SensorConstants.getBeamBreakComponent());
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

    public Command intakeAndAutoStow() {
        return withRequirement(
            Commands.sequence(
                intake().withDeadline(beamBreak.stateWaitDebounced(true)), // Intake until the Beam Break sees a game piece
                Commands.waitSeconds(0.2), // Wait a short time to make sure game piece is fully processed
                stow()
            )
        );
    }
}
