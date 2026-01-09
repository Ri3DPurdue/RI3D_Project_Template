package frc.lib.util;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;

/**
 * Utility class for creating and configuring motor controller configurations.
 * Provides helper methods for setting up SparkMax and TalonFX motor controllers
 * with common settings like gearing, current limits, and soft limits.
 */
public class ConfigUtil {
    /** Default safe supply current limit in amps for motor controllers */
    public static final double safeCurrentLimitAmps = 40.0;
    
    /** Default safe stator current limit in amps for TalonFX motor controllers */
    public static final double safeStatorCurrentLimitAmps = 60.0;

    /**
     * Configures a SparkMax with gear ratio conversion factors.
     * Sets both velocity and position conversion factors based on the gearing ratio.
     * 
     * @param config The SparkMax configuration to modify
     * @param gearing The gear ratio (motor rotations per mechanism rotation)
     * @return The modified configuration for method chaining
     */
    public static SparkMaxConfig withGearing(SparkMaxConfig config, double gearing) {
        config.encoder.velocityConversionFactor(1.0 / gearing);
        config.encoder.positionConversionFactor(1.0 / gearing);

        return config;
    }

    /**
     * Configures a SparkMax with smart current limiting.
     * 
     * @param config The SparkMax configuration to modify
     * @param currentLimit The current limit in amps (note: currently uses safeCurrentLimitAmps instead)
     * @return The modified configuration for method chaining
     */
    public static SparkMaxConfig withCurrentLimits(SparkMaxConfig config, double currentLimit) {
        config.smartCurrentLimit((int) safeCurrentLimitAmps);

        return config;
    }

    /**
     * Creates a safe SparkMax configuration with specified gearing and current limit.
     * 
     * @param gearing The gear ratio (motor rotations per mechanism rotation)
     * @param currentLimit The current limit in amps
     * @return A new SparkMax configuration with safe defaults
     */
    public static SparkMaxConfig getSafeMaxConfig(double gearing, double currentLimit) {
        return withCurrentLimits(withGearing(new SparkMaxConfig(), gearing), currentLimit);
    }

    /**
     * Creates a safe SparkMax configuration with specified gearing and default current limit.
     * Uses the safeCurrentLimitAmps constant for the current limit.
     * 
     * @param gearing The gear ratio (motor rotations per mechanism rotation)
     * @return A new SparkMax configuration with safe defaults
     */
    public static SparkMaxConfig getSafeMaxConfig(double gearing) {
        return withCurrentLimits(withGearing(new SparkMaxConfig(), gearing), safeCurrentLimitAmps);
    }

    /**
     * Configures a SparkMax with software-based position limits.
     * Enables both forward and reverse soft limits to prevent mechanism damage.
     * 
     * @param config The SparkMax configuration to modify
     * @param forwardSoftLimit The maximum forward position limit
     * @param reverseSoftLimit The maximum reverse position limit
     * @return The modified configuration for method chaining
     */
    public static SparkMaxConfig withSoftLimits(SparkMaxConfig config, Angle forwardSoftLimit, Angle reverseSoftLimit) {
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.forwardSoftLimit(forwardSoftLimit.in(Rotations));
        config.softLimit.reverseSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(reverseSoftLimit.in(Rotations));

        return config;
    }

    /**
     * Configures a TalonFX with a gear ratio for sensor feedback.
     * Sets the sensor-to-mechanism ratio for accurate position and velocity reporting.
     * 
     * @param config The TalonFX configuration to modify
     * @param gearing The gear ratio (motor rotations per mechanism rotation)
     * @return The modified configuration for method chaining
     */
    public static TalonFXConfiguration withGearing(TalonFXConfiguration config, double gearing) {
        config.Feedback.SensorToMechanismRatio = gearing;

        return config;
    }

    /**
     * Configures a TalonFX with both supply and stator current limits.
     * Supply current is the current drawn from the battery, stator current is the current through the motor.
     * 
     * @param config The TalonFX configuration to modify
     * @param supplyCurrentLimit The supply (battery) current limit in amps
     * @param statorCurrentLimit The stator (motor) current limit in amps
     * @return The modified configuration for method chaining
     */
    public static TalonFXConfiguration withCurrentLimits(TalonFXConfiguration config, double supplyCurrentLimit, double statorCurrentLimit) {
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = supplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        return config;
    }

    /**
     * Creates a safe TalonFX configuration with specified gearing and current limits.
     * 
     * @param gearing The gear ratio (motor rotations per mechanism rotation)
     * @param supplyCurrentLimit The supply (battery) current limit in amps
     * @param statorCurrentLimit The stator (motor) current limit in amps
     * @return A new TalonFX configuration with safe defaults
     */
    public static TalonFXConfiguration getSafeFXConfig(double gearing, double supplyCurrentLimit, double statorCurrentLimit) {
        return withCurrentLimits(withGearing(new TalonFXConfiguration(), gearing), supplyCurrentLimit, statorCurrentLimit);
    }

    /**
     * Creates a safe TalonFX configuration with specified gearing and default current limits.
     * Uses safeCurrentLimitAmps and safeStatorCurrentLimitAmps constants.
     * 
     * @param gearing The gear ratio (motor rotations per mechanism rotation)
     * @return A new TalonFX configuration with safe defaults
     */
    public static TalonFXConfiguration getSafeFXConfig(double gearing) {
        return withCurrentLimits(withGearing(new TalonFXConfiguration(), gearing), safeCurrentLimitAmps, safeStatorCurrentLimitAmps);
    }

    /**
     * Configures a TalonFX with software-based position limits.
     * Enables both forward and reverse soft limits to prevent mechanism damage.
     * 
     * @param config The TalonFX configuration to modify
     * @param forwardSoftLimit The maximum forward position limit
     * @param reverseSoftLimit The maximum reverse position limit
     * @return The modified configuration for method chaining
     */
    public static TalonFXConfiguration withSoftLimits(TalonFXConfiguration config, Angle forwardSoftLimit, Angle reverseSoftLimit) {
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftLimit.in(Rotations);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftLimit.in(Rotations);

        return config;
    }
}
