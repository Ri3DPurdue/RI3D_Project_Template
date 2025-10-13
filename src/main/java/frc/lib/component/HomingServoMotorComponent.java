package frc.lib.component;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Util.UnitsUtil;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.Setpoint;

public class HomingServoMotorComponent<M extends MotorIO> extends ServoMotorComponent<M> {
    private boolean homing = false;
    private boolean needsToHome = true;
    private HomingConfig homingConfig;
    private Debouncer homingDebouncer;


    public HomingServoMotorComponent(M motorIO, Angle epsilon, HomingConfig config) {
        super(motorIO, epsilon);
        homingConfig = config;
        homingDebouncer = new Debouncer(config.homingDebouce.in(Units.Seconds), DebounceType.kRising);
    }

    private boolean setpointNearHome() {
        return getSetpoint().outputType.isPositionControl() && positionNearHome(BaseUnits.AngleUnit.of(getSetpoint().value));
    }

    private boolean positionNearHome(Angle position) {
        return UnitsUtil.isNear(homingConfig.homePosition, position, epsilonThreshold);
    }

    private boolean isNearHome() {
        return positionNearHome(getPosition());
    }

    @Override
    public void periodic() {
        super.periodic();
        if (needsToHome && setpointNearHome() && isNearHome()) { // If homing is needed, targeting the homing location, and almost there
            beginHomingSequence(); // Start the homing sequence
        }
        if (homing && DriverStation.isEnabled()) { // If homing (and enabled so the voltage is ACTUALLY being applied)
            if (homingDebouncer.calculate(
                    getVelocity().abs(BaseUnits.AngleUnit.per(BaseUnits.TimeUnit)) <= 
                    homingConfig.homingVelocity.baseUnitMagnitude())) { // If you've been under the homing velocity threshold for the debounce (if you've stopped)
                resetPosition(homingConfig.homePosition); // You know you're at the home position so reset it
                applySetpoint(Setpoint.profiledPositionSetpoint(homingConfig.homePosition)); // Target the homing location with position control so you don't keep slamming into it (this also ends homing sequence because new setpoint is applied)
            }
        }
    }

    @Override
    public void applySetpoint(Setpoint setpoint) {
        super.applySetpoint(setpoint); // Apply your setpoint
        if (homing) { // If you were in the middle of homing
            endHomingSequence(); // Cancel homing
        }
        if (!setpointNearHome()) { // If you're leaving your home position
            needsToHome = true; // You now need to rezero
        }
    }

    public void beginHomingSequence() {
        homing = true; // Save that you are currently homing
        useSoftLimits(false); // Disable soft limits so you can physically hit the hardstop
        super.applySetpoint(Setpoint.voltageSetpoint(homingConfig.homingVoltage)); // Go at your homing voltage (but don't apply setpoint normally so homing isn't canceled)
        homingDebouncer.calculate(false); // Reset the debouncer
    }

    public void endHomingSequence() {
        homing = false; // Save that you're done homing
        useSoftLimits(true); // Turn soft limits back on
    }


    public static class HomingConfig {
        public Angle homePosition = BaseUnits.AngleUnit.zero();
        public AngularVelocity homingVelocity = BaseUnits.AngleUnit.zero().div(BaseUnits.TimeUnit.zero());
        public Voltage homingVoltage = BaseUnits.VoltageUnit.zero();
        public Time homingDebouce = BaseUnits.TimeUnit.zero();
    }
    
}
