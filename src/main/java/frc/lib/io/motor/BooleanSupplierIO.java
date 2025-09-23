package frc.lib.io.motor;

import java.util.function.BooleanSupplier;

public class BooleanSupplierIO extends DigitalIO {
    private final BooleanSupplier input;

    public BooleanSupplierIO(double debounceSeconds, BooleanSupplier supplier) {
        super(debounceSeconds);
        input = supplier;
    }

    @Override
    public boolean get() {
        return input.getAsBoolean();
    }
}
