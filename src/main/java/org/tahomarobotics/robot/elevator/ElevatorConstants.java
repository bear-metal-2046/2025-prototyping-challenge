package org.tahomarobotics.robot.elevator;

public final class ElevatorConstants {
    private ElevatorConstants() {}

    // placeholder conversion factor
    public static final double ROTATIONS_PER_METER = 10000.0;
    public static double metersToRotations(double m) { return m * ROTATIONS_PER_METER; }

    // placeholder preset heights (meters)
    public static final double HEIGHT_LOW_M = 0.35;
    public static final double HEIGHT_MID_M = 0.75;
    public static final double HEIGHT_HIGH_M = 1.35;

    // Native positions
    public static final double POS_LOW = metersToRotations(HEIGHT_LOW_M);
    public static final double POS_MID = metersToRotations(HEIGHT_MID_M);
    public static final double POS_HIGH = metersToRotations(HEIGHT_HIGH_M);
}
