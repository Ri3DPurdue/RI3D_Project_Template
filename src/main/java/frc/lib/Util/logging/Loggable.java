package frc.lib.Util.logging;

public interface Loggable {
    public void log(String path);

    public default void log(String subdirectory, String name) {
        log(subdirectory + "/" + name);
    }
}