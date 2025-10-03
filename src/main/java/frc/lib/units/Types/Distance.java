package frc.lib.units.Types;

import frc.lib.units.Value;

public class Distance extends Value<DistanceUnit, Distance>{
    public Distance(double baseUnitsValue) {
        super(baseUnitsValue);
    }

    public Distance(double val, DistanceUnit unit) {
        super(val, unit);
    }

    @Override
    public Distance make(double baseUnitsValue) {
        return new Distance(baseUnitsValue);
    }
    
    public Angle withRadius(Distance radius) {
        return new Angle(radius.getBaseValue(), AngleUnit.radians);
    }
}
