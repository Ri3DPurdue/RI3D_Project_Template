package frc.lib.units.Types;

import frc.lib.units.Unit;

public class Distance extends Unit {
    public Distance(double multiplierToBase, String name) {
        super(multiplierToBase, name);
    }
    
    public static Distance meters = new Distance(1.0, "Meters");
    public static Distance centimeters = new Distance(100.0, "Centimeters");
}
