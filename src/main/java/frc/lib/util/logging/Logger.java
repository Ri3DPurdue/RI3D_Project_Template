package frc.lib.util.logging;

import dev.doglog.DogLog;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.struct.StructSerializable;

public class Logger extends DogLog {
    // Loggable

    /*
     * Logs the given loggable value out to the given key
     */
    public static void log(String key, Loggable value) {
        value.log(key);
    }

    /*
     * Logs the given loggable out to the given path with the given name
     */
    public static void log(String path, String key, Loggable value) {
        log(path + "/" + key, value);
    }

    /*
     * Logs out an array to the given key
     * Creates new subdirectories with the given index
     */
    public static void log(String key, Loggable[] values) {
        for (int i = 0; i < values.length; i++) {
            log(key, "" + i, values[i]);
        }
    }

    /*
     * Logs out an array to the given path with the given name
     * Creates new subdirectories with the given index
     */
    public static void log(String path, String key, Loggable[] values){
        log(path + "/" + key, values);
    }

    // Measure

    /*
     * Logs out the given measure in the given unit to the given key
     * Creates two sub-values for the value itself and the unit
     */
    public static <U extends Unit> void log(String key, Measure<U> value, U unit) {
        log(key, "Value", value.in(unit));
        log(key, "Units", unit.name());
    }

    /*
     * Logs out the given measure in it's base unit to the given key
     */
    @SuppressWarnings("unchecked")
    public static void log(String key, Measure<?> value) {
        log(key, (Measure<Unit>) value, value.baseUnit());
    }

    /*
     * Logs out the given measure to the given path with the given name
     * in the given unit
     */
    public static <U extends Unit> void log(String path, String key, Measure<U> value, U unit) {
        log(path + "/" + key, value, unit);
    }

    /*
     * Logs out the given measure to the given path with the given name
     * in the unit's base unit
     */
    public static void log(String path, String key, Measure<?> value) {
        log(path + "/" + key, value);
    }

    /*
     * Logs out the measures to the given key
     */
    public static <U extends Unit> void log(String key, Measure<U>[] values, U unit) {
        for (int i = 0; i  < values.length; i++){
            log(key + "/" + i, values[i], unit);
        }
    }

    /*
     * Logs out the measures to the given key
     */
    public static void log(String key, Measure<?> values[]) {
        for (int i = 0; i < values.length; i++) {
            log(key + "/" + i, values[i]);
        }
    }

    /*
     * Logs out the measures to the given path with the given name
     */
    public static <U extends Unit> void log(String path, String key, Measure<U>[] values, U unit) {
        log(path + "/" + key, values, unit);
    }

    /*
     * Logs out the measures to the given path with the given name
     */
    public static void log(String path, String key, Measure<?>[] values) {
        log(path + "/" + key, values);
    }

    // boolean

    /*
     * Logs out the boolean to the given path with the given name
     */
    public static void log(String path, String key, boolean value) {
        log(path + "/" + key, value);
    }

    /*
     * Logs out the booleans to the given path with the given name
     */
    public static void log(String path, String key, boolean[] value) {
        log(path + "/" + key, value);
    }

    // double

    /*
     * Logs out the double to the given path with the given name
     */
    public static void log(String path, String key, double value) {
        log(path + "/" + key, value);
    }

    /*
     * Logs out the doubles to the given path with the given name
     */
    public static void log(String path, String key, double[] value) {
        log(path + "/" + key, value);
    }

    // int

    /*
     * Logs out the int to the given path with the given name
     */
    public static void log(String path, String key, int value) {
        log(path + "/" + key, value);
    }

    /*
     * Logs out the ints to the given path with the given name
     */
    public static void log(String path, String key, int[] value) {
        log(path + "/" + key, value);
    }

    // String

    /*
     * Logs out the string to the given path with the given name
     */
    public static void log(String path, String key, String value) {
        log(path + "/" + key, value);
    }

    /*
     * Logs out the strings to the given path with the given name
     */
    public static void log(String path, String key, String[] value) {
        log(path + "/" + key, value);
    }

    // Struct

    /*
     * Logs out the serializable structure to the given path with the given name
     */
    public static <T extends StructSerializable> void log(String path, String key, T value) {
        log(path + "/" + key, value);
    }

    /*
     * Logs out the serializable structures to the given path with the given name
     */
    public static <T extends StructSerializable> void log(String path, String key, T[] value) {
        log(path + "/" + key, value);
    }

    // Enum

    /*
     * Logs out the enum to the given path with the given name
     */
    public static <E extends Enum<E>> void log(String path, String key, E value) {
        log(path + "/" + key, value);
    }

    /*
     * Logs out the enums to the given path with the given name
     */
    public static <E extends Enum<E>> void log(String path, String key, E[] value) {
        log(path + "/" + key, value);
    }
}
