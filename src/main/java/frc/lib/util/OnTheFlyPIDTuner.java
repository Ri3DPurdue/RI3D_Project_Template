package frc.lib.util;

import java.util.Set;
import java.util.function.UnaryOperator;

import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.hardware.motor.SparkBaseIO;
import frc.lib.io.motor.MotorIO;

public class OnTheFlyPIDTuner<M extends MotorIO> {
    private final M io;
    private final String prefix;
    private final PIDParams defaults;
    private final UnaryOperator<Pair<M, PIDParams>> applier;

    private static final String tunablePKey = "P";
    private static final String tunableIKey = "I";
    private static final String tunableDKey = "D";
    private static final String tunableSKey = "S";
    private static final String tunableGKey = "G";
    private static final String tunableGravityUsingArmCosineKey = "Gravity Using Arm Cosine";
    private static final String applyKey = "Apply";

    public OnTheFlyPIDTuner(M ioToConfig, String networkTablesPrefix, PIDParams defaultPID, UnaryOperator<Pair<M, PIDParams>> configApplier) {
        io = ioToConfig;
        prefix = networkTablesPrefix;
        defaults = defaultPID;
        applier = configApplier;

        SmartDashboard.putNumber(prefix + tunablePKey, defaults.p);
        SmartDashboard.putNumber(prefix + tunableIKey, defaults.i);
        SmartDashboard.putNumber(prefix + tunableDKey, defaults.d);
        SmartDashboard.putNumber(prefix + tunableSKey, defaults.f);
        SmartDashboard.putNumber(prefix + tunableGKey, defaults.g);
        SmartDashboard.putBoolean(prefix + tunableGravityUsingArmCosineKey, defaults.gravityUsingArmCosine);   
        SmartDashboard.putData(prefix + applyKey, applyConfig());
    }

    public Command applyConfig() {
        return Commands.defer(() -> Commands.runOnce(() -> apply()), Set.of());
    }

    public void apply() {
        applier.apply(Pair.of(io, getNewParams()));
    }

    public PIDParams getNewParams() {
        PIDParams newPID = new PIDParams();

        newPID.p = SmartDashboard.getNumber(prefix + tunablePKey, defaults.p);
        newPID.i = SmartDashboard.getNumber(prefix + tunableIKey, defaults.i);
        newPID.d = SmartDashboard.getNumber(prefix + tunableDKey, defaults.d);
        newPID.f = SmartDashboard.getNumber(prefix + tunableSKey, defaults.f);
        newPID.g = SmartDashboard.getNumber(prefix + tunableGKey, defaults.g);
        newPID.gravityUsingArmCosine = SmartDashboard.getBoolean(prefix + tunableGravityUsingArmCosineKey, defaults.gravityUsingArmCosine);

        return newPID;
    }

    public static UnaryOperator<Pair<SparkBaseIO, PIDParams>> getSparkApplier() {
        return (Pair<SparkBaseIO, PIDParams> pair) -> {
            pair.getFirst().changeConfig((SparkBaseConfig c) -> {
                c.closedLoop.pidf(
                    pair.getSecond().p, 
                    pair.getSecond().i, 
                    pair.getSecond().d, 
                    pair.getSecond().f
                    // Sparks do not support gravity config
                );

                return c;
            });

            return pair;
        };
    }

    public static class PIDParams {
        public double p = 0.0;
        public double i = 0.0;
        public double d = 0.0;
        public double f = 0.0;
        public double g = 0.0;
        public boolean gravityUsingArmCosine = false;
    }
}
