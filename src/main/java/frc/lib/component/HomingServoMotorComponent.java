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
import frc.lib.io.motor.setpoints.*;

/**
 * A servo motor component with automatic homing functionality.
 * 
 * <p>This component extends {@link ServoMotorComponent} to add automatic homing capabilities.
 * Homing is the process of moving a mechanism to a known physical hard stop and resetting
 * the encoder position, ensuring accurate absolute positioning throughout operation.
 * 
 * <p>The homing sequence is triggered automatically when:
 * <ul>
 * <li>The mechanism needs homing (after moving away from home)</li>
 * <li>A setpoint targets the home position</li>
 * <li>The mechanism is near the home position</li>
 * </ul>
 * 
 * <p>During homing, the component:
 * <ol>
 * <li>Disables soft limits to allow movement to the hard stop</li>
 * <li>Applies a constant voltage towards the hard stop</li>
 * <li>Monitors velocity to detect when the mechanism has stopped</li>
 * <li>Resets the encoder position once stopped</li>
 * <li>Re-enables soft limits and applies position control</li>
 * </ol>
 * 
 * @param <M> the type of MotorIO implementation used by this component
 * @see ServoMotorComponent
 * @see HomingConfig
 */
public class HomingServoMotorComponent<M extends MotorIO> extends ServoMotorComponent<M> {
    /** Whether a homing sequence is currently in progress */
    private boolean homing = false;
    /** Whether the mechanism needs to be homed before reliable operation */
    private boolean needsToHome = true;
    /** Configuration parameters for the homing sequence */
    private HomingConfig homingConfig;
    /** Debouncer to ensure velocity has stabilized before declaring homing complete */
    private Debouncer homingDebouncer;


    /**
     * Constructs a new HomingServoMotorComponent with homing configuration.
     * 
     * @param motorIO the motor I/O implementation to control
     * @param epsilon the position tolerance threshold for target detection
     * @param startAngle the initial position to set on the encoder
     * @param config the homing configuration parameters
     */
    public HomingServoMotorComponent(M motorIO, Angle epsilon, Angle startAngle, HomingConfig config) {
        super(motorIO, epsilon, startAngle);
        homingConfig = config;
        homingDebouncer = new Debouncer(config.homingDebouce.in(Units.Seconds), DebounceType.kRising);
    }

    /**
     * Checks if the current setpoint targets a position near home.
     * 
     * @return true if the setpoint is a position setpoint near home, false otherwise
     */
    private boolean setpointNearHome() {
        if (getSetpoint() instanceof PositionSetpoint p) {
            return positionNearHome(p.get());
        } else {
            return false;
        }
    }

    /**
     * Checks if a given position is within tolerance of the home position.
     * 
     * @param position the position to check
     * @return true if the position is near home, false otherwise
     */
    private boolean positionNearHome(Angle position) {
        return UnitsUtil.isNear(homingConfig.homePosition, position, epsilonThreshold);
    }

    /**
     * Checks if the mechanism's current position is near home.
     * 
     * @return true if the current position is near home, false otherwise
     */
    private boolean isNearHome() {
        return positionNearHome(getPosition());
    }

    /**
     * Performs periodic updates including automatic homing sequence management.
     * 
     * <p>This method handles:
     * <ul>
     * <li>Initiating homing when needed and conditions are met</li>
     * <li>Monitoring velocity during homing to detect completion</li>
     * <li>Resetting encoder position once homing is complete</li>
     * <li>Transitioning from voltage control to position control after homing</li>
     * </ul>
     */
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
                applySetpoint(new ProfiledPositionSetpoint(homingConfig.homePosition)); // Target the homing location with position control so you don't keep slamming into it (this also ends homing sequence because new setpoint is applied)
            }
        }
    }

    /**
     * Applies a setpoint and manages homing state accordingly.
     * 
     * <p>This method:
     * <ul>
     * <li>Cancels any in-progress homing sequence</li>
     * <li>Marks the mechanism as needing homing if moving away from home</li>
     * </ul>
     * 
     * @param setpoint the setpoint to apply
     */
    @Override
    public void applySetpoint(BaseSetpoint<?> setpoint) {
        super.applySetpoint(setpoint); // Apply your setpoint
        if (homing) { // If you were in the middle of homing
            endHomingSequence(); // Cancel homing
        }
        if (!setpointNearHome()) { // If you're leaving your home position
            needsToHome = true; // You now need to rezero
        }
    }

    /**
     * Begins the homing sequence.
     * 
     * <p>This method:
     * <ul>
     * <li>Disables soft limits to allow movement to the hard stop</li>
     * <li>Applies a constant voltage towards the home position</li>
     * <li>Resets the velocity debouncer</li>
     * <li>Sets the homing flag</li>
     * </ul>
     */
    public void beginHomingSequence() {
        homing = true; // Save that you are currently homing
        useSoftLimits(false); // Disable soft limits so you can physically hit the hardstop
        super.applySetpoint(new VoltageSetpoint(homingConfig.homingVoltage)); // Go at your homing voltage (but don't apply setpoint normally so homing isn't canceled)
        homingDebouncer.calculate(false); // Reset the debouncer
    }

    /**
     * Ends the homing sequence and restores normal operation.
     * 
     * <p>This method:
     * <ul>
     * <li>Clears the homing flag</li>
     * <li>Re-enables soft limits for safe operation</li>
     * </ul>
     */
    public void endHomingSequence() {
        homing = false; // Save that you're done homing
        useSoftLimits(true); // Turn soft limits back on
    }


    /**
     * Configuration parameters for the homing sequence.
     * 
     * <p>This class defines all parameters needed to execute a safe and reliable homing sequence:
     * <ul>
     * <li><b>homePosition</b> - The encoder position to set when the hard stop is reached</li>
     * <li><b>homingVelocity</b> - The velocity threshold below which the mechanism is considered stopped</li>
     * <li><b>homingVoltage</b> - The voltage to apply during homing (towards the hard stop)</li>
     * <li><b>homingDebouce</b> - How long velocity must stay below threshold to confirm stopping</li>
     * </ul>
     */
    public static class HomingConfig {
        /** The encoder position value to set when homing is complete */
        public Angle homePosition = BaseUnits.AngleUnit.zero();
        /** The velocity threshold below which the mechanism is considered stopped */
        public AngularVelocity homingVelocity = BaseUnits.AngleUnit.zero().div(BaseUnits.TimeUnit.zero());
        /** The voltage to apply while moving towards the hard stop */
        public Voltage homingVoltage = BaseUnits.VoltageUnit.zero();
        /** The time velocity must remain below threshold before declaring homing complete */
        public Time homingDebouce = BaseUnits.TimeUnit.zero();
    }
    
}
