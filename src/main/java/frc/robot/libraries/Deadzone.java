package frc.robot.libraries;

public class Deadzone {
    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double invlerp(double a, double b, double v) {
        return (v - a) / (b - a);
    }

    /**
     * Fixed, previous version didnt work with negative values, stupid me
     * there is literally nothing wrong with this code, it's a you problem
     */
    public static double deadZone(double value, double deadzone) {
        if (Math.abs(value) > deadzone)
            return Math.signum(value) * invlerp(deadzone, 1.0, Math.abs(value));
        else
            return 0.0D;
    }

    public static double cutOff(double value, double cutoff) {
        if (Math.abs(value) < cutoff)
            return 0;
        else
            return value;
    }

}
