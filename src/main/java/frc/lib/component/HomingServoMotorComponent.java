package frc.lib.component;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.Setpoint;
import frc.lib.io.motor.Setpoint.Type;
import frc.lib.util.Util;

public class HomingServoMotorComponent<M extends MotorIO> extends ServoMotorComponent<M> {
    private boolean homing = false;
    private boolean needsToHome = true;
    private HomingConfig homingConfig;
    private Debouncer homingDebouncer;


    public HomingServoMotorComponent(M motorIO, double epsilon, HomingConfig config) {
        super(motorIO, epsilon);
        homingConfig = config;
        homingDebouncer = new Debouncer(config.homingDebouce, DebounceType.kRising);
    }

    private boolean setpointNearHome() {
        return getSetpoint().outputType.isPositionControl() && positionNearHome(getSetpoint().value);
    }

    private boolean positionNearHome(double position) {
        return Util.epsilonEquals(position, homingConfig.homePosition, epsilonThreshold);
    }

    private boolean isNearHome() {
        return positionNearHome(getPosition());
    }

    @Override
    public void periodic() {
        super.periodic();
        if (needsToHome && setpointNearHome() && isNearHome()) {
            beginHomingSequence();
        }
        if (homing && DriverStation.isEnabled()) {
            if (homingDebouncer.calculate(Math.abs(getVelocity()) >= homingConfig.homingVelocity)) {
                zeroPosition(homingConfig.homePosition);
                applySetpoint(new Setpoint(Type.ProfiledPosition, homingConfig.homePosition));
                endHomingSequence();
            }
        }
    }

    @Override
    public void applySetpoint(Setpoint setpoint) {
        super.applySetpoint(setpoint);
        if(homing) {
            endHomingSequence();
        }
        if(!setpointNearHome()) {
            needsToHome = true;
        }
    }

    public void beginHomingSequence() {
        homing = true;
        useSoftLimits(false);
        super.applySetpoint(new Setpoint(Type.Voltage, homingConfig.homingVoltage));
        homingDebouncer.calculate(false);
    }

    public void endHomingSequence() {
        homing = false;
        useSoftLimits(true);
    }


    public class HomingConfig {
        public double homePosition = 0.0; // Mechanims radians or meters
        public double homingVelocity = 0.0; // Mechanism radians or meters per second
        public double homingVoltage = 0.0; // Volts
        public double homingDebouce = 0.0; // Seconds
    }
    
}
