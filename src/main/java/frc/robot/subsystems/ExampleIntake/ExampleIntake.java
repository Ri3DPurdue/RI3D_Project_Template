package frc.robot.subsystems.ExampleIntake;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.ServoMotorComponent;
import frc.lib.hardware.motor.SparkBaseIO;
import frc.lib.io.motor.Setpoint;

public class ExampleIntake extends ComponentSubsystem {
    private final ServoMotorComponent<SparkBaseIO> pivot;

    public ExampleIntake() {
        pivot = registerComponent(ExampleIntakeConstants.Pivot.getPivot(), "Pivot");
    }

    public Command intake() {
        return pivot.applySetpointCommand(Setpoint.voltageSetpoint(Units.Volts.of(12.0)));
    }
}
