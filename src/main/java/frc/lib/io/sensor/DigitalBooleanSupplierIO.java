package frc.lib.io.sensor;

import java.util.function.BooleanSupplier;

public class DigitalBooleanSupplierIO extends DigitalIO {
    private final BooleanSupplier input;

    public DigitalBooleanSupplierIO(double debounceSeconds, BooleanSupplier supplier) {
        super(debounceSeconds);
        input = supplier;
    }

    @Override
    public boolean get() {
        return input.getAsBoolean();
    }
}
