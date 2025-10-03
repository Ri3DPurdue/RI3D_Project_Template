package frc.lib.units;

public class Value<U extends Unit> {
    private double baseValue;

    public Value(double val, U unit) {
        baseValue = val * unit.multiplierToBase;
    }

    public Value(double baseUnitsValue) {
        baseValue = baseUnitsValue;
    }

    public double getBaseValue() {
        return baseValue;
    }

    public Value<U> plus(Value<U> other) {
        return new Value<>(baseValue + other.baseValue);
    }

    public Value<U> minus(Value<U> other) {
        return new Value<>(baseValue - other.baseValue);
    }

    public Value<U> times(double scalar) {
        return new Value<>(baseValue * scalar);
    }

    public Value<U> divide(double divisor) {
        return new Value<>(baseValue / divisor);
    }
}
