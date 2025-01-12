package frc.robot;

// WPILib alreaday claimed MathUtil.
public class MathHelper {
    public static final double kEpsilon = 1e-12;

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }
}
