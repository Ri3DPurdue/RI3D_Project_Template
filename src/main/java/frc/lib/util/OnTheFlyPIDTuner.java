package frc.lib.util;

import java.util.Set;
import java.util.function.UnaryOperator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.io.motor.BaseConfig;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.BaseConfig.PIDParams;

public class OnTheFlyPIDTuner {
    private final MotorIO io;
    private final String prefix;
    private PIDParams defaults;
    private static final String tunablePKey = "P";
    private static final String tunableIKey = "I";
    private static final String tunableDKey = "D";
    private static final String tunableSKey = "S";
    private static final String tunableGKey = "G";
    private static final String tunableGravityUsingArmCosineKey = "Gravity Using Arm Cosine";
    private static final String applyKey = "Apply";

    public OnTheFlyPIDTuner(MotorIO ioToConfig, String networkTablesPrefix, PIDParams defaultPID) {
        io = ioToConfig;
        prefix = networkTablesPrefix;

        SmartDashboard.putNumber(prefix + tunablePKey, defaults.p);
        SmartDashboard.putNumber(prefix + tunableIKey, defaults.i);
        SmartDashboard.putNumber(prefix + tunableDKey, defaults.d);
        SmartDashboard.putNumber(prefix + tunableSKey, defaults.f);
        SmartDashboard.putNumber(prefix + tunableGKey, defaults.g);
        SmartDashboard.putBoolean(tunableGravityUsingArmCosineKey, defaults.gravityUsingArmCosine);   
        SmartDashboard.putData(prefix + applyKey, applyConfig());
    }

    public Command applyConfig() {
        return Commands.defer(() -> {
            return Commands.runOnce(() -> {
                PIDParams newPID = new PIDParams();

                newPID.p = SmartDashboard.getNumber(prefix + tunablePKey, defaults.p);
                newPID.i = SmartDashboard.getNumber(prefix + tunableIKey, defaults.i);
                newPID.d = SmartDashboard.getNumber(prefix + tunableDKey, defaults.d);
                newPID.f = SmartDashboard.getNumber(prefix + tunableSKey, defaults.f);
                newPID.g = SmartDashboard.getNumber(prefix + tunableGKey, defaults.g);
                newPID.gravityUsingArmCosine = SmartDashboard.getBoolean(tunableGravityUsingArmCosineKey, defaults.gravityUsingArmCosine);

                UnaryOperator<BaseConfig> configChanger = (BaseConfig config) -> {
                    config.pid = newPID;
                    return config;
                };

                io.changeConfig(configChanger);

                });
        }, Set.of());
    }
}
