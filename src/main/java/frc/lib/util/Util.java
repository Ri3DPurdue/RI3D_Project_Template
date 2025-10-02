package frc.lib.util;

public class Util {
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return Math.abs(a - b) <= epsilon;
    }
}
