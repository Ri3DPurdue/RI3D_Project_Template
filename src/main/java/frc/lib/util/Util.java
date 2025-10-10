package frc.lib.util;

public static class Util {
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return Math.abs(a - b) <= epsilon;
    }
}
