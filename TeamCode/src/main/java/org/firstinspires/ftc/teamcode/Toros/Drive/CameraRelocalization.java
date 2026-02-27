package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Camera-based robot pose estimate from AprilTag (for Dashboard / comparison with odometry).
 *
 * FTC SDK: ftcPose X,Y,Z and Range use the tag library's distance unit (CenterStage = METER, others often INCH).
 * Angles (bearing, elevation, pitch, roll, yaw) are in degrees.
 * Camera frame: X = right, Y = forward from lens, Z = up.
 *
 * With camera tilted 15° up: horizontal component of camera→tag uses cos(15°) on Y and sin(15°) on Z so
 * horizontal forward = Y*cos(15°) - Z*sin(15°). Then rotate by robot heading to get field-frame vector;
 * robot pose = tag position minus that vector.
 */
@Config
public class CameraRelocalization {
    /** Camera tilt above horizontal (degrees). Positive = camera aimed up. */
    public static double CAMERA_TILT_DEG = 15.0;

    /** True if tag library uses meters (e.g. CenterStageTagLibrary); false for inches. */
    public static boolean TAG_LIBRARY_USES_METERS = true;

    private static final double M_TO_IN = 39.37007874;

    /** Field positions of AprilTags (inches). Center Stage backdrop. */
    public static double TAG_20_X = 60.0;
    public static double TAG_20_Y = 72.0;
    public static double TAG_24_X = -60.0;
    public static double TAG_24_Y = 72.0;

    /**
     * Robot pose (field frame, inches) from one AprilTag detection and known tag field position.
     * Uses odometry heading to convert camera→tag from robot frame to field frame.
     *
     * @param d                 AprilTag detection (ftcPose in library units: X right, Y forward, Z up)
     * @param tagFieldXInches   Tag field X (inches)
     * @param tagFieldYInches   Tag field Y (inches)
     * @param robotHeadingRad   Robot heading from odometry (radians). RR convention: 0 = +X, CCW positive.
     */
    public static Pose2d robotPoseFromTag(AprilTagDetection d, double tagFieldXInches, double tagFieldYInches, double robotHeadingRad) {
        // --- 1. Camera→tag in camera frame: X,Y,Z (same units as tag library) ---
        double x = d.ftcPose.x;
        double y = d.ftcPose.y;
        double z = d.ftcPose.z;
        if (TAG_LIBRARY_USES_METERS) {
            x *= M_TO_IN;
            y *= M_TO_IN;
            z *= M_TO_IN;
        }
        // Now x,y,z in inches: right, forward from camera, up.

        // --- 2. 15° tilt: project onto horizontal (field) plane. Camera Y is tilted up by CAMERA_TILT_DEG. ---
        // Horizontal forward = Y*cos(tilt) - Z*sin(tilt); horizontal right = X.
        double tiltRad = Math.toRadians(CAMERA_TILT_DEG);
        double yHorz = y * Math.cos(tiltRad) - z * Math.sin(tiltRad);
        double xHorz = x;
        // (xHorz, yHorz) = camera→tag in robot horizontal plane (right, forward) in inches.

        // --- 3. Rotate to field frame. Heading 0 = +X, CCW positive: forward = (cos(h), sin(h)), right = (-sin(h), cos(h)). ---
        double c = Math.cos(robotHeadingRad);
        double s = Math.sin(robotHeadingRad);
        double fieldDx = -xHorz * s + yHorz * c;
        double fieldDy = xHorz * c + yHorz * s;
        // (fieldDx, fieldDy) = camera→tag in field frame (inches).

        // --- 4. Robot position = tag position minus camera→tag (camera is on robot). ---
        double robotX = tagFieldXInches - fieldDx;
        double robotY = tagFieldYInches - fieldDy;

        return new Pose2d(robotX, robotY, robotHeadingRad);
    }

    /** Get tag field position by ID (inches). */
    public static Vector2d getTagFieldPosition(int tagId) {
        if (tagId == 20) return new Vector2d(TAG_20_X, TAG_20_Y);
        if (tagId == 24) return new Vector2d(TAG_24_X, TAG_24_Y);
        return null;
    }
}
