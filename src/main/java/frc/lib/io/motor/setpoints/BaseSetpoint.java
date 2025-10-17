package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public abstract class BaseSetpoint<M extends Measure<? extends Unit>> {
    M value;

    public BaseSetpoint(M value) {
        this.value = value;
    }

    public M get() {
        return value;
    }
}
