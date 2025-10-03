package frc.lib.units;

public abstract class Value<U extends Unit, V extends Value<U, V>> {
    private double baseValue;

    public Value(double val, U unit) {
        baseValue = val * unit.multiplierToBase;
    }

    public Value(double baseUnitsValue) {
        baseValue = baseUnitsValue;
    }

    public abstract V make(double baseUnitsValue);

    public double getBaseValue() {
        return baseValue;
    }

    public double in(U u) {
        return getBaseValue() / u.multiplierToBase;
    }

    public V plus(V other) {
        return make(getBaseValue() + other.getBaseValue());
    }

    public V minus(V other) {
        return make(getBaseValue() - other.getBaseValue());
    }

    public V times(double scalar) {
        return make(getBaseValue() * scalar);
    }

    public V divide(double divisor) {
        return make(getBaseValue() / divisor);
    }
}
