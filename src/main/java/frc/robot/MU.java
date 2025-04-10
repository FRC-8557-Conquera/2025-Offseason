package frc.robot;

public class MU {
    public static boolean isNear(double a, double b, double tol) {
        return Math.abs(a-b) <= tol;
    }
}
