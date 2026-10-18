package frc.robot.subsystems.exampleShooter;

import frc.lib.component.DigitalIOComponent;
import frc.lib.io.sensor.digital.DigitalBooleanSupplierIO;
import frc.lib.io.sensor.digital.DigitalIO;
import frc.lib.io.sensor.digital.DigitalInIO;
import frc.robot.IDs;
import frc.robot.Robot;
import frc.robot.controlBoard.ControlBoardConstants;

public class SensorConstants {
    // Seconds of pure values of a certain reading before asertion (filters out momentary signal flickers)
    private static final double feederBeamBreakDebounceSeconds = 0.1;

    public static final DigitalIOComponent getFeederBeamBreakComponent() {
        return new DigitalIOComponent(getBeamBreakIO());
    }

    public static final DigitalIO getBeamBreakIO() {
        return Robot.isReal()
         ? new DigitalInIO(feederBeamBreakDebounceSeconds, IDs.SHOOTER_FEEDER_BEAM_BREAK.id)
         : new DigitalBooleanSupplierIO(feederBeamBreakDebounceSeconds, () -> ControlBoardConstants.driver.povRight().getAsBoolean()); // Simulate the breambreak with a controller button
    }
}
