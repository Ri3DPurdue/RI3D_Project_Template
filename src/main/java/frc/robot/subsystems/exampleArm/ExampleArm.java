package frc.robot.subsystems.exampleArm;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.HomingServoMotorComponent;
import frc.lib.component.MotorComponent;
import frc.lib.component.ServoMotorComponent;
import frc.lib.io.motor.rev.SparkBaseIO;
import frc.lib.util.logging.Logger;
import frc.lib.util.visualization.AngularNode;
import frc.lib.util.visualization.LinearNode;
import frc.lib.util.visualization.PositionNode;

public class ExampleArm extends ComponentSubsystem {
    private final ServoMotorComponent<SparkBaseIO> wrist;
    private final HomingServoMotorComponent<SparkBaseIO> elevator;
    private final MotorComponent<SparkBaseIO> endEffector;
    private final AngularNode wristNode;
    private final LinearNode elevatorNode;

    public ExampleArm() {
        wrist = registerComponent("Wrist", WristConstants.getComponent());
        elevator = registerComponent("Elevator", ElevatorConstants.getComponent());
        endEffector = registerComponent("EndEffector", EndEffectorConstants.getComponent());
        elevatorNode = new LinearNode(
            "Elevator Node", 
            PositionNode.BASE, 
            new Transform3d(0.06, 0.05, 0.07, new Rotation3d(Degrees.of(0), Degrees.of(-90), Degrees.of(0))), 
            elevator,
            ElevatorConstants.converter
        );
        wristNode = new AngularNode(
            "Wrist Node", 
            elevatorNode, 
            Transform3d.kZero, 
            wrist
        );
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

    public Command stow() {
        return withRequirement(
            Commands.parallel(
                wrist.applyPositionSetpointCommandWithWait(WristConstants.stowSetpoint),
                elevator.applyPositionSetpointCommandWithWait(ElevatorConstants.stowSetpoint),
                endEffector.applySetpointCommand(EndEffectorConstants.idleSetpoint)
            )
        );
    }

    @Override
    public void log(String name) {
        super.log(name);
        Logger.log(name, "Wrist Node", wristNode);
        Logger.log(name, "Elevator Node", elevatorNode);
    }
}
