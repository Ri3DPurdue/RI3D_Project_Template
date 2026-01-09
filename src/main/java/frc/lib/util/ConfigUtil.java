package frc.lib.util;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;

public class ConfigUtil {
    public static final double safeCurrentLimitAmps = 40.0;
    public static final double safeStatorCurrentLimitAmps = 60.0;

    public static SparkMaxConfig withGearing(SparkMaxConfig config, double gearing) {
        config.encoder.velocityConversionFactor(1.0 / gearing);
        config.encoder.positionConversionFactor(1.0 / gearing);

        return config;
    }

    public static SparkMaxConfig withCurrentLimits(SparkMaxConfig config, double currentLimit) {
        config.smartCurrentLimit((int) safeCurrentLimitAmps);

        return config;
    }

    public static SparkMaxConfig getSafeMaxConfig(double gearing, double currentLimit) {
        return withCurrentLimits(withGearing(new SparkMaxConfig(), gearing), currentLimit);
    }

    public static SparkMaxConfig getSafeMaxConfig(double gearing) {
        return withCurrentLimits(withGearing(new SparkMaxConfig(), gearing), safeCurrentLimitAmps);
    }

    public static SparkMaxConfig withSoftLimits(SparkMaxConfig config, Angle forwardSoftLimit, Angle reverseSoftLimit) {
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.forwardSoftLimit(forwardSoftLimit.in(Rotations));
        config.softLimit.reverseSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(reverseSoftLimit.in(Rotations));

        return config;
    }

    public static TalonFXConfiguration withGearing(TalonFXConfiguration config, double gearing) {
        config.Feedback.SensorToMechanismRatio = gearing;

        return config;
    }

    public static TalonFXConfiguration withCurrentLimits(TalonFXConfiguration config, double supplyCurrentLimit, double statorCurrentLimit) {
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = supplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        return config;
    }

    public static TalonFXConfiguration getSafeFXConfig(double gearing, double supplyCurrentLimit, double statorCurrentLimit) {
        return withCurrentLimits(withGearing(new TalonFXConfiguration(), gearing), supplyCurrentLimit, statorCurrentLimit);
    }

    public static TalonFXConfiguration getSafeFXConfig(double gearing) {
        return withCurrentLimits(withGearing(new TalonFXConfiguration(), gearing), safeCurrentLimitAmps, safeStatorCurrentLimitAmps);
    }

    public static TalonFXConfiguration withSoftLimits(TalonFXConfiguration config, Angle forwardSoftLimit, Angle reverseSoftLimit) {
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftLimit.in(Rotations);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftLimit.in(Rotations);

        return config;
    }
}
