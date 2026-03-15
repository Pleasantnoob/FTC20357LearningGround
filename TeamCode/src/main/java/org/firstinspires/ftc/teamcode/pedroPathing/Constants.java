package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Pedro Pathing constants derived from existing Road Runner tuning.
 * Motor names match config (fl, bl, fr, br). Pinpoint pod positions in inches from robot center;
 * encoder directions match RR PinpointLocalizer (par REVERSED, perp FORWARD).
 */
public final class Constants {

    // ----- Source: RR MecanumDrive.Params (converted where applicable) -----
    /** Inches per encoder tick (RR inPerTick). Used for Pinpoint offset conversion only. */
    public static final double IN_PER_TICK = 0.00194723378259707;
    /** RR maxWheelVel (in/s) → Pedro x/y velocity. */
    public static final double MAX_VELOCITY = 100.0;
    /** RR path gains (axial 14, lateral 10, heading 9) scaled for Pedro translational/heading PID. */
    public static final double TRANSLATIONAL_P = 0.03;
    public static final double TRANSLATIONAL_F = 0.015;
    public static final double HEADING_P = 0.8;
    public static final double HEADING_F = 0.01;
    public static final double DRIVE_P = 0.1;
    public static final double DRIVE_D = 0.00035;
    public static final double DRIVE_T = 0.6;
    public static final double DRIVE_F = 0.015;
    /** RR feedforward kS ≈ 0.9; Pedro uses filtered drive PID. Mass (kg) for dynamics. */
    public static final double ROBOT_MASS_KG = 16.2;
    /** Zero-power decel (tuned or from RR profile). */
    public static final double FORWARD_ZERO_POWER_ACCEL = -25.93;
    public static final double LATERAL_ZERO_POWER_ACCEL = -67.34;

    // ----- Source: RR PinpointLocalizer.Params -----
    /** Par encoder offset (ticks) → inches for reference. Pedro uses pod position; measure from center. */
    public static final double PAR_Y_TICKS = -648.1950127810029;
    public static final double PERP_X_TICKS = -3692.469269014993;
    /** Pod position in inches from robot center (Y = forward, X = strafe). Tune if pose is wrong. */
    public static final double FORWARD_POD_Y_IN = IN_PER_TICK * PAR_Y_TICKS;  // ~ -1.26
    public static final double STRAFE_POD_X_IN = IN_PER_TICK * PERP_X_TICKS; // ~ -7.19

    private Constants() {}

    public static final FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants()
            .mass(ROBOT_MASS_KG)
            .forwardZeroPowerAcceleration(FORWARD_ZERO_POWER_ACCEL)
            .lateralZeroPowerAcceleration(LATERAL_ZERO_POWER_ACCEL)
            .translationalPIDFCoefficients(new PIDFCoefficients(TRANSLATIONAL_P, 0, 0, TRANSLATIONAL_F))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.4, 0, 0.005, 0.0006))
            .headingPIDFCoefficients(new PIDFCoefficients(HEADING_P, 0, 0, HEADING_F))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.1, 0.0005))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(DRIVE_P, 0, DRIVE_D, DRIVE_T, DRIVE_F))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.000005, 0.6, 0.01))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005);

    public static final MecanumConstants DRIVE_CONSTANTS = new MecanumConstants()
            .leftFrontMotorName("fl")
            .leftRearMotorName("bl")
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(MAX_VELOCITY)
            .yVelocity(MAX_VELOCITY);

    public static final PinpointConstants LOCALIZER_CONSTANTS = new PinpointConstants()
            .forwardPodY(FORWARD_POD_Y_IN)
            .strafePodX(STRAFE_POD_X_IN)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    /** tValue, velocity, translational, heading, timeout, brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart. */
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
            0.995, 0.1, 0.1, 0.009, 50, 1.25, 10, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(FOLLOWER_CONSTANTS, hardwareMap)
                .mecanumDrivetrain(DRIVE_CONSTANTS)
                .pinpointLocalizer(LOCALIZER_CONSTANTS)
                .pathConstraints(PATH_CONSTRAINTS)
                .build();
    }
}
