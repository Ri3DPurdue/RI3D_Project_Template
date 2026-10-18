package frc.robot.subsystems.exampleClimber;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.ServoMotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;
import frc.lib.util.logging.Logger;
import frc.lib.util.visualization.LinearNode;
import frc.lib.util.visualization.PositionNode;

public class ExampleClimber extends ComponentSubsystem {
    private final ServoMotorComponent<TalonFXIO> climber;
    private final LinearNode climberNode;

    public ExampleClimber() {
        climber = registerComponent("Climber", ClimberConstants.getComponent());
        climberNode = new LinearNode(
            "Climber Node", 
            PositionNode.BASE, 
            new Transform3d(-0.255, 0, 0.04, new Rotation3d(Degrees.of(0), Degrees.of(-90), Degrees.of(0))), 
            climber,
            ClimberConstants.converter
        );
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

    @Override
    public void log(String name) {
        super.log(name);
        Logger.log(name, "Climber Node", climberNode);
    }
}
