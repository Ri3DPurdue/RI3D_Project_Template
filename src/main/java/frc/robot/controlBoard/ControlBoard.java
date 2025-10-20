package frc.robot.controlBoard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;

public class ControlBoard {
    public static final CommandXboxController driver = new CommandXboxController(0);
    public static void bindControls(Superstructure s) {
        driver.a().onTrue(s.intake());
        driver.b().onTrue(s.spit());
        driver.x().onTrue(s.stow());
    }
}
