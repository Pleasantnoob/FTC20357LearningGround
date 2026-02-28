package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Camera-based robot pose estimate from AprilTag (for Dashboard / comparison with odometry).
 *
 * ftcPose X,Y,Z are in MILLIMETERS (SDK CenterStage); we convert to inches. Angles in degrees.
 * Camera frame: X = right, Y = forward from lens, Z = up.
 *
 * With camera tilted 15° up: horizontal component uses cos(15°) on Y and sin(15°) on Z.
 * Robot pose = tag position minus camera→tag in field frame; then subtract camera mounting offset.
 */
@Config
public class CameraRelocalization {
    private static final double MM_TO_IN = 1.0 / 25.4;

    /** Camera tilt above horizontal (degrees). Positive = camera aimed up. */
    public static double CAMERA_TILT_DEG = 15.0;

    /** AprilTag ID used for relocalization (blue goal only for now). Center Stage blue backdrop. */
    public static int BLUE_GOAL_TAG_ID = 20;

    /**
     * Blue goal tag field position (inches). Must match your field frame: blue in (-,-) quadrant uses negative X,Y (e.g. -70, -64).
     * Origin at center; +X = right from red wall, +Y = away from red wall. Tune on Dashboard to match MainDrive goal.
     */
    public static double BLUE_GOAL_TAG_X = -70.0;
    public static double BLUE_GOAL_TAG_Y = -64.0;

    /** Camera mounting: inches forward and right of robot center (when turret aligned with robot heading). Applied so pose is robot center, not camera lens. */
    public static double CAMERA_FORWARD_OFFSET = 6.0;
    public static double CAMERA_RIGHT_OFFSET = 0.0;

    /** Optional small offset (inches) for final tune to match Pinpoint. */
    public static double CAMERA_OFFSET_X = 0.0;
    public static double CAMERA_OFFSET_Y = 0.0;

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
        // --- 1. Camera→tag in camera frame: ftcPose X,Y,Z are MILLIMETERS (SDK CenterStage) → convert to inches ---
        double x = d.ftcPose.x * MM_TO_IN;
        double y = d.ftcPose.y * MM_TO_IN;
        double z = d.ftcPose.z * MM_TO_IN;
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

        // --- 4. Camera position = tag position minus camera→tag. Then convert to robot center using mounting offset. ---
        double robotX = tagFieldXInches - fieldDx;
        double robotY = tagFieldYInches - fieldDy;

        // --- 5. Camera is forward/right of robot center: subtract that offset in field frame. ---
        double ch = Math.cos(robotHeadingRad), sh = Math.sin(robotHeadingRad);
        robotX -= CAMERA_FORWARD_OFFSET * ch;
        robotY -= CAMERA_FORWARD_OFFSET * sh;
        robotX += CAMERA_RIGHT_OFFSET * sh;
        robotY -= CAMERA_RIGHT_OFFSET * ch;

        // --- 6. Optional small calibration offset. ---
        robotX -= CAMERA_OFFSET_X;
        robotY -= CAMERA_OFFSET_Y;

        return new Pose2d(robotX, robotY, robotHeadingRad);
    }

    /** Get tag field position by ID (inches). Only blue goal tag supported for relocalization. */
    public static Vector2d getTagFieldPosition(int tagId) {
        if (tagId == BLUE_GOAL_TAG_ID) return new Vector2d(BLUE_GOAL_TAG_X, BLUE_GOAL_TAG_Y);
        return null;
    }
}
