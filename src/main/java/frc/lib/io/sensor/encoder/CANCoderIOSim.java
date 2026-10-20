package frc.lib.io.sensor.encoder;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;

public class CANCoderIOSim extends CANCoderIO {
    private Supplier<Angle> position;
    public CANCoderIOSim(int canID, Supplier<Angle> simulatedPosition) {
        this(canID, "", simulatedPosition);
    }

    public CANCoderIOSim(int canID, String canbus, Supplier<Angle> simulatedPosition) {
        super(canID, canbus);
        position = simulatedPosition;
    }

    @Override
    public void periodic() {
        canCoder.getSimState().setRawPosition(position.get());
        super.periodic();
    }
    
}
