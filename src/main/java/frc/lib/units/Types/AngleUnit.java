package frc.lib.units.Types;

import frc.lib.units.Unit;

public class AngleUnit extends Unit{
    public AngleUnit(double multiplierToBase, String name) {
        super(multiplierToBase, name);
    }    
    
    public static AngleUnit radians = new AngleUnit(1.0, "Radians");
    public static AngleUnit rotations = new AngleUnit(2.0 * Math.PI, "Rotations");
}
