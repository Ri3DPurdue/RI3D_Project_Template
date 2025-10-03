package frc.lib.units.Types;

import frc.lib.units.Unit;

public class Angle extends Unit{
    public Angle(double multiplierToBase, String name) {
        super(multiplierToBase, name);
    }    
    
    public static Angle radians = new Angle(1.0, "Radians");
    public static Angle rotations = new Angle(2.0 * Math.PI, "Rotations");
}
