package frc.lib.util.logging;

public interface Loggable {
    /*
     * Logs this value out to the given path
     */
    public void log(String path);

    /*
     * Logs this value out to the subdirectory under the given name
     */
    public default void log(String subdirectory, String name) {
        log(subdirectory + "/" + name);
    }
}