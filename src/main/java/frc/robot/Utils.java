package frc.robot;

/**
 * Utilility functions.
 */
public final class Utils {

    /* Check doubles are equal to within epsilon preceision */
    public static boolean isDoubleEqual(double a, double b, double epsilon) {
        return Math.abs(a - b) < epsilon;
    }

    public static double rescale(double value, double oldMin, double oldMax, double newMin, double newMax) {
        return ((value - oldMin) / (oldMax - oldMin)) * (newMax - newMin) + newMin;
    }
}
