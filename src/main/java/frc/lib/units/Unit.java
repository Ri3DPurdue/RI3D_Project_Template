package frc.lib.units;

public abstract class Unit {
    public final double multiplierToBase; 
    public final String name; 


    protected Unit(double multiplierToBase, String name) {
        this.multiplierToBase = multiplierToBase;
        this.name = name;
    }
}
