package org.firstinspires.ftc.teamcode.hardware;

public class Angle {
    private static final double TAU = Math.PI * 2;

    /**
     * Returns angle clamped to [0, 2pi].
     *
     * @param angle angle measure in radians
     * @return normalized angle
     */
    public static double norm(double angle) {
        double modifiedAngle = angle % TAU;
        modifiedAngle = (modifiedAngle + TAU) % TAU;
        return modifiedAngle;
    }

    /**
     * Returns angleDelta clamped to [-pi, pi].
     *
     * @param angleDelta angle delta in radians
     * @return normalized angle delta
     */
    public static double normDelta(double angleDelta) {
        double modifiedAngleDelta = norm(angleDelta);
        if (modifiedAngleDelta > Math.PI) {
            modifiedAngleDelta -= TAU;
        }
        return modifiedAngleDelta;
    }
}
