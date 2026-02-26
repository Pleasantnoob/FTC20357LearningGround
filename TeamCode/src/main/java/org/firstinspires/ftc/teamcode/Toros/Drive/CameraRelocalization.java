package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Camera-based pose estimate for Dashboard display only (no fusion yet).
 *
 * Flow: camera gives POLAR (distance in m, bearing°, elevation°)
 *       → apply 15° camera tilt to get horizontal distance (field plane only)
 *       → convert to CARTESIAN (x, y) in inches
 *       → robot pose = tag position minus camera-to-tag vector (all in inches on FTC field).
 */
@Config
public class CameraRelocalization {
    /** Camera tilt above horizontal (degrees). Positive = camera aimed up. Used to get horizontal distance only. */
    public static double CAMERA_TILT_DEG = 15.0;

    /** Meters to inches. */
    private static final double M_TO_IN = 39.37;

    /** Field positions of AprilTags (inches). Center Stage backdrop tags. */
    public static double TAG_20_X = 60.0;
    public static double TAG_20_Y = 72.0;
    public static double TAG_24_X = -60.0;
    public static double TAG_24_Y = 72.0;

    /** Scale for camera Y to match odometry (e.g. 0.5 if camera Y was ~2x). */
    public static double FIELD_Y_SCALE = 0.5;

    /**
     * From one AprilTag detection: polar (range m, bearing°, elevation°) → horizontal distance (15° tilt) → Cartesian inches.
     * Result is robot position in inches for FTC field display.
     */
    public static Pose2d robotPoseFromTag(AprilTagDetection d, double tagFieldXInches, double tagFieldYInches) {
        // --- 1. Polar input: range (tag library often uses INCHES; if > 10 assume inches), angles in degrees ---
        double rawRange = d.ftcPose.range;
        double rangeM = (rawRange > 10) ? (rawRange / M_TO_IN) : rawRange;
        double bearingDeg = d.ftcPose.bearing;
        double elevationDeg = d.ftcPose.elevation;

        // --- 2. Camera frame: vector from camera to tag (right, up, forward) in meters ---
        double radB = Math.toRadians(bearingDeg);
        double radE = Math.toRadians(elevationDeg);
        double cosE = Math.cos(radE);
        double sinE = Math.sin(radE);
        double camRightM = rangeM * cosE * Math.sin(radB);
        double camUpM = rangeM * sinE;
        double camForwardM = rangeM * cosE * Math.cos(radB);

        // --- 3. 15° tilt: project onto field horizontal so we use horizontal distance only ---
        // Camera is tilted CAMERA_TILT_DEG up: field horizontal forward = camForward*cos(tilt) - camUp*sin(tilt)
        double tiltRad = Math.toRadians(CAMERA_TILT_DEG);
        double fieldRightM = camRightM;
        double fieldForwardM = camForwardM * Math.cos(tiltRad) - camUpM * Math.sin(tiltRad);

        // --- 4. Convert to Cartesian in inches (camera and field both in inches) ---
        double fieldRightIn = fieldRightM * M_TO_IN;
        double fieldForwardIn = fieldForwardM * M_TO_IN;

        // Robot = tag minus (camera→tag in field plane). FTC field: negate so camera pose matches odometry sign convention.
        double robotX = -(tagFieldXInches - fieldRightIn);
        double robotY = -(tagFieldYInches - fieldForwardIn);
        // Y from camera often ~2x odometry; scale down if needed (1.0 = no change)
        robotY *= FIELD_Y_SCALE;

        return new Pose2d(robotX, robotY, 0);
    }

    /** Get tag field position by ID (inches). */
    public static Vector2d getTagFieldPosition(int tagId) {
        if (tagId == 20) return new Vector2d(TAG_20_X, TAG_20_Y);
        if (tagId == 24) return new Vector2d(TAG_24_X, TAG_24_Y);
        return null;
    }
}
