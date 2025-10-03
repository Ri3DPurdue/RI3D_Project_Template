package frc.lib.units.Types;

import frc.lib.units.Unit;

public class DistanceUnit extends Unit {
    public DistanceUnit(double multiplierToBase, String name) {
        super(multiplierToBase, name);
    }
    
    public static DistanceUnit meters = new DistanceUnit(1.0, "Meters");
    public static DistanceUnit centimeters = new DistanceUnit(100.0, "Centimeters");
}
