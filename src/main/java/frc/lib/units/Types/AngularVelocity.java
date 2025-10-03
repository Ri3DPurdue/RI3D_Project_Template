package frc.lib.units.Types;

import frc.lib.units.Value;

public class AngularVelocity extends Value<AngularVelocityUnit, AngularVelocity> {
    public AngularVelocity(double baseUnitsValue) {
        super(baseUnitsValue);
    }

    public AngularVelocity(double val, AngularVelocityUnit unit) {
        super(val, unit);
    }

    @Override
    public AngularVelocity make(double baseUnitsValue) {
        return new AngularVelocity(baseUnitsValue);
    }

    public Angle times(Time time) {
        return new Angle(getBaseValue() * time.getBaseValue());
    }
    
}
