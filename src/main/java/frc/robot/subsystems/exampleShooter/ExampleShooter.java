package frc.robot.subsystems.exampleShooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.FlywheelMotorComponent;
import frc.lib.component.MotorComponent;
import frc.lib.component.ServoMotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;

public class ExampleShooter extends ComponentSubsystem {
    private final ServoMotorComponent<TalonFXIO> hood;
    private final MotorComponent<TalonFXIO> feeder;
    private final FlywheelMotorComponent<TalonFXIO> flywheel;

    public ExampleShooter() {
        hood = registerComponent("Hood", HoodConstants.getComponent());
        feeder = registerComponent("Feeder", FeederConstants.getComponent());
        flywheel = registerComponent("Flywheel", FlywheelConstants.getComponent());
    }

    public Command prepShot() {
        return withRequirement(
            Commands.parallel(
                feeder.applySetpointCommand(FeederConstants.idleSetpoint),
                hood.applyPositionSetpointCommandWithWait(HoodConstants.shotSetpoint),
                flywheel.applyVelocitySetpointCommandWithWait(FlywheelConstants.shotSetpoint)
            )
        );
    }

    public Command fire() {
        return withRequirement(
            feeder.applySetpointCommand(FeederConstants.feedSetpoint)
        );
    }

    public Command fireWhenReady() {
        return withRequirement(
            Commands.sequence(
                prepShot(),
                fire()
            )
        );
    }

    public Command stow() {
        return withRequirement(
            Commands.parallel(
                feeder.applySetpointCommand(FeederConstants.idleSetpoint),
                hood.applySetpointCommand(HoodConstants.stowSetpoint),
                flywheel.applySetpointCommand(FlywheelConstants.idleSetpoint)
            )
        );
    }
}