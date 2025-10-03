package frc.lib.units.Types;

import frc.lib.units.Value;

public class Time extends Value<TimeUnit, Time>{

    public Time(double baseUnitsValue) {
        super(baseUnitsValue);
        //TODO Auto-generated constructor stub
    }

    public Time(double val, TimeUnit unit) {
        super(val, unit);
    }

    @Override
    public Time make(double baseUnitsValue) {
        return new Time(baseUnitsValue);
    }
    
}
