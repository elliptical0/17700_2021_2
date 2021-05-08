package org.firstinspires.ftc.teamcode;

public class Constants {
    /**
     * 0.0 to 1.0, multiplier applied to motor power.
     * Compensates for hardware differences in motor speeds.
     * Order: Front-left, front-right, rear-right, rear-left.
     */
    public static final double[] MOTORCALIB = {1, 1, 1, 1};

    /**
     * Multiplier applied to motor speeds when holding down the left stick.
     */
    public static final double SLOWSPEED = 0.35;

    /**
     * Deadwheel radius.
     * old: 0.995370749373427
     * new: 0.9899713817476206
     */
    public static final double DEADWHEEL_RADIUS = 0.9899713817476206;

    /**
     * Encoder ticks per revolution.
     */
    public static final double TICKS_PER_REV = 8192;

    /**
     * Either 1 or -1, used to reverse certain encoders.
     */
    public static final int[] ENCODER_DIRECTIONS = {1, 1, -1};

    /**
     * Distance between the two lateral dead wheels.
     * old: 11.440378084618675
     * new: 15.054853749727858
     */
    public static final double LATERAL_DISTANCE = 15.054853749727858;

    /**
     * Distance between the rear dead wheel and the center.
     * old: 8.9560151267944225
     * new: 2.3276239445354205
     */
    public static final double REAR_OFFSET = 2.3276239445354205;

    /**
     * Number of spins during LATERAL_DISTANCE and REAR_OFFSET calibration. Higher = more accurate.
     */
    public static final double CALIB_SPINS = 14;

    /**
     * Number of inches traveled during DEADWHEEL_RADIUS calculation. Higher = more accurate.
     */
    public static final double CALIB_DIST = 48;

    /**
     * Shooting transform in inches.
     */
    public static final Transform SHOOTING_T = new Transform(68.2, 22.157, 0.1); //0.059
    //90.94, 14.72, 0.50

    /**
     * Starting transform of the robot in inches.
     */
    public static final Transform[] STARTING_T = {new Transform(9, 15, 0), new Transform(9, 39, 0)};

    /**
     *
     */
    public static final double[] LAUNCH_AIM_POSITIONS = {0.4, 0.25};

    /**
     *
     */
    public static final double[] COUNTERWEIGHT_POSITIONS = {1, 0.3333};

    /**
     *
     */
    public static final double[] WOBBLE_AIM_POSITIONS = {0.5, 0.65, 0.8};

    public static final double[] WOBBLE_HAND_POSITIONS = {0.15, 0.7};

    /**
     * Deadzones for when the robot is autonomously seeking a position.
     */
    public static final double DEADZONE_POS = 0.75;
    public static final double DEADZONE_ANGLE = Math.toRadians(6);

    public static final double DEADZONE_SERVO = 0.1;

    public static final boolean SERVOS_ACTIVE = true;

    public static final boolean FLYWHEEL_ENCODER = true;

    public static final double FLYWHEEL_MAX_RPM = 845;

    public static final double FILTER_MIN_AREA = 0;

    public static final double FILTER_HUE_HIGH = 48;

    public static final double FILTER_SATURATION_LOW = 57;

    public static final double FILTER_VALUE_LOW = 0;

    public static final double FILTER_RATIO = 1.6;

    public static final String BUILD_NAME = "v0.14.2";
}