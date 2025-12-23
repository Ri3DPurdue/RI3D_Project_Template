package frc.robot.subsystems.exampleIntake;

import frc.lib.component.DigitalIOComponent;
import frc.lib.io.sensor.DigitalBooleanSupplierIO;
import frc.lib.io.sensor.DigitalIO;
import frc.lib.io.sensor.DigitalInIO;
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
         : new DigitalBooleanSupplierIO(beamBreakDebounceSeconds, ControlBoardConstants.driver.povUp()); // Simulate the breambreak with the povUp() button
    }
}
