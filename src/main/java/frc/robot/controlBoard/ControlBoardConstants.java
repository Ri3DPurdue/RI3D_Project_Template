package frc.robot.controlBoard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControlBoardConstants {
    public static final CommandXboxController driver = new CommandXboxController(0);
    public static final CommandXboxController operator = new CommandXboxController(1);

    public static final double triggerThreshold = 0.2;
}
