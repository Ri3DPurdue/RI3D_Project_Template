package frc.lib.units.Types;

import frc.lib.units.Unit;

public class AngularVelocityUnit extends Unit {
    protected AngularVelocityUnit(AngleUnit angle, TimeUnit time) {
        super(angle.multiplierToBase / time.multiplierToBase, angle.name + " per " + time.name);
    }
    
}
