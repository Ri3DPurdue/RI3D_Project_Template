package frc.robot.subsystems.exampleIntake;

import frc.lib.component.DigitalIOComponent;
import frc.lib.io.sensor.digital.DigitalBooleanSupplierIO;
import frc.lib.io.sensor.digital.DigitalIO;
import frc.lib.io.sensor.digital.DigitalInIO;
import frc.robot.IDs;
import frc.robot.Robot;
import frc.robot.controlBoard.ControlBoardConstants;

public class SensorConstants {
    // Seconds of pure values of a certain reading before asertion (filters out momentary signal flickers)
    private static final double beamBreakDebounceSeconds = 0.1;

    public static final DigitalIOComponent getBeamBreakComponent() {
        return new DigitalIOComponent(getBeamBreakIO());
    }

    public static final DigitalIO getBeamBreakIO() {
        return Robot.isReal()
         ? new DigitalInIO(beamBreakDebounceSeconds, IDs.INTAKE_BEAM_BREAK.id)
         : new DigitalBooleanSupplierIO(beamBreakDebounceSeconds, ControlBoardConstants.driver.povLeft()); // Simulate the breambreak with a controller button
    }
}
