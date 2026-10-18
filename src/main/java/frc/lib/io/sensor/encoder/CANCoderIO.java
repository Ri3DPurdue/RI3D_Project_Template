package frc.lib.io.sensor.encoder;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class CANCoderIO extends EncoderIO {
    private CANcoder canCoder;
    private MagnetSensorConfigs magCfg;

    public CANCoderIO(int canID, String bus) {
        this.canCoder = new CANcoder(canID, bus);
        this.magCfg = new MagnetSensorConfigs();
        this.magCfg.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        this.canCoder.getConfigurator().apply(this.magCfg);
    }

    public CANCoderIO(int canID) {
        this.canCoder = new CANcoder(canID);
        this.magCfg = new MagnetSensorConfigs();
        this.magCfg.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        this.canCoder.getConfigurator().apply(this.magCfg);
    }

    @Override
    public Angle getPosition() {
        return canCoder.getPosition().getValue();
    }

    @Override
    public void setPosition(Angle position) {
        canCoder.setPosition(position);
    }

    @Override
    public void setOffset(Angle offset) {
        magCfg.MagnetOffset = -(offset.in(Units.Rotations) % 1.0);
        canCoder.getConfigurator().apply(magCfg);
    }

    @Override
    public void setInverted(boolean inverted) {
        magCfg.SensorDirection = inverted ?
            SensorDirectionValue.Clockwise_Positive :
            SensorDirectionValue.CounterClockwise_Positive;
        canCoder.getConfigurator().apply(magCfg);
    }
}
