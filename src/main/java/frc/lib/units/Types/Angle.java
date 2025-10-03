package frc.lib.units.Types;

import frc.lib.units.Value;

public class Angle extends Value<AngleUnit, Angle>{
    public Angle(double val, AngleUnit unit) {
        super(val, unit);
    }

    public Angle(double baseUnitsValue) {
        super(baseUnitsValue);
    }

    @Override
    public Angle make(double baseUnitsValue) {
        return new Angle(baseUnitsValue);
    }

    public AngularVelocity per(Time time) {
        return new AngularVelocity(getBaseValue() / time.getBaseValue());
    }
}
