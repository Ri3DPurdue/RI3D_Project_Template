package frc.robot.subsystems.exampleIntake.util.logging;

import dev.doglog.DogLog;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.struct.StructSerializable;

public class Logger extends DogLog {
    // Loggable

    public static void log(String key, Loggable value) {
        value.log(key);
    }

    public static void log(String path, String key, Loggable value) {
        log(path + "/" + key, value);
    }

    public static void log(String key, Loggable[] values) {
        for (int i = 0; i < values.length; i++) {
            log(key, "" + i, values[i]);
        }
    }

    public static void log(String path, String key, Loggable[] values){
        log(path + "/" + key, values);
    }

    // Measure

    public static <U extends Unit> void log(String key, Measure<U> value, U unit) {
        log(key, "Value", value.in(unit));
        log(key, "Units", unit.name());
    }

    public static <U extends Unit> void log(String key, Measure<U> value) {
        log(key, value, value.baseUnit());
    }

    public static <U extends Unit> void log(String path, String key, Measure<U> value, U unit) {
        log(path + "/" + key, value, unit);
    }

    public static void log(String path, String key, Measure<?> value) {
        log(path + "/" + key, value);
    }

    public static <U extends Unit> void log(String key, Measure<U>[] values, U unit) {
        for (Measure<U> value : values) {
            log(key, value, unit);
        }
    }

    public static void log(String key, Measure<?> values[]) {
        for (Measure<?> value : values) {
            log(key, value);
        }
    }

    public static <U extends Unit> void log(String path, String key, Measure<U>[] values, U unit) {
        for (Measure<U> value : values) {
            log(path, key, value, unit);
        }
    }

    public static void log(String path, String key, Measure<?>[] values) {
        for (Measure<?> value : values) {
            log(path, key, value);
        }
    }

    // boolean

    public static void log(String path, String key, boolean value) {
        log(path + "/" + key, value);
    }

    public static void log(String path, String key, boolean[] value) {
        log(path + "/" + key, value);
    }

    // double

    public static void log(String path, String key, double value) {
        log(path + "/" + key, value);
    }

    public static void log(String path, String key, double[] value) {
        log(path + "/" + key, value);
    }

    // int

    public static void log(String path, String key, int value) {
        log(path + "/" + key, value);
    }

    public static void log(String path, String key, int[] value) {
        log(path + "/" + key, value);
    }

    // String

    public static void log(String path, String key, String value) {
        log(path + "/" + key, value);
    }

    public static void log(String path, String key, String[] value) {
        log(path + "/" + key, value);
    }

    // Struct

    public static <T extends StructSerializable> void log(String path, String key, T value) {
        log(path + "/" + key, value);
    }

    public static <T extends StructSerializable> void log(String path, String key, T[] value) {
        log(path + "/" + key, value);
    }

    // Enum

    public static void log(String path, String key, Enum<?> value) {
        log(path + "/" + key, value);
    }

    public static void log(String path, String key, Enum<?>[] value) {
        log(path + "/" + key, value);
    }
}
