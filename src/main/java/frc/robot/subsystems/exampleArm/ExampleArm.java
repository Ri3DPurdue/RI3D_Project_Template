package frc.robot.subsystems.exampleArm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.MotorComponent;
import frc.lib.component.ServoMotorComponent;
import frc.lib.io.motor.rev.SparkBaseIO;

public class ExampleArm extends ComponentSubsystem {
    private final ServoMotorComponent<SparkBaseIO> wrist;
    private final ServoMotorComponent<SparkBaseIO> elevator;
    private final MotorComponent<SparkBaseIO> endEffector;

    public ExampleArm() {
        wrist = registerComponent("Wrist", WristConstants.getComponent());
        elevator = registerComponent("Elevator", ElevatorConstants.getComponent());
        endEffector = registerComponent("EndEffector", EndEffectorConstants.getComponent());
    }

    public Command prepScore() {
        return withRequirement(
            Commands.parallel(
                wrist.applyPositionSetpointCommandWithWait(WristConstants.scoreSetpoint),
                elevator.applyPositionSetpointCommandWithWait(ElevatorConstants.scoreSetpoint)
            )
        );
    }

    public Command score() {
        return withRequirement(
            endEffector.applySetpointCommand(EndEffectorConstants.outtakeSetpoint)
        );
    }
}
