package org.firstinspires.ftc.teamcode.Toros.Drive;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RR.Drawing;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * TeleOp for testing camera relocalization only.
 * Runs camera + AprilTag and Pinpoint odometry; displays both poses so you can compare.
 * Pinpoint = odometry (pods + IMU). Camera = pose from AprilTag 20/24 + CameraRelocalization.
 */
@TeleOp(name = "Camera Reloc Test", group = "Toros")
public class CameraRelocTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private MecanumDrive mecanumDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Camera Reloc Test — Pinpoint vs camera pose");
        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        initAprilTag();

        // Pinpoint odometry (same start as MainDrive default)
        double startX = -48.0, startY = -50.0, startHeadingDeg = -90.0;
        Pose2d initialPose = new Pose2d(startX, startY, Math.toRadians(startHeadingDeg));
        mecanumDrive = new MecanumDrive(hardwareMap, initialPose);
        mecanumDrive.localizer.setPose(initialPose);

        telemetry.addLine("Init done. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            mecanumDrive.updatePoseEstimate();
            Pose2d pinpointPose = mecanumDrive.localizer.getPose();
            double headingRad = pinpointPose.heading.toDouble();

            // Camera relocalization: blue goal tag only
            Pose2d cameraPose = null;
            int tagId = -1;
            for (AprilTagDetection d : aprilTag.getDetections()) {
                if (d.metadata == null) continue;
                if (d.id != CameraRelocalization.BLUE_GOAL_TAG_ID) continue;
                Vector2d tagPos = CameraRelocalization.getTagFieldPosition(d.id);
                if (tagPos != null) {
                    cameraPose = CameraRelocalization.robotPoseFromTag(d, tagPos.x, tagPos.y, headingRad);
                    tagId = d.id;
                    break;
                }
            }

            // --- Telemetry ---
            telemetry.addLine("--- Pinpoint (odometry) ---");
            telemetry.addData("Pinpoint x (in)", "%.2f", pinpointPose.position.x);
            telemetry.addData("Pinpoint y (in)", "%.2f", pinpointPose.position.y);
            telemetry.addData("Pinpoint heading (deg)", "%.1f", Math.toDegrees(headingRad));

            telemetry.addLine("");
            telemetry.addLine("--- Camera relocalization ---");
            if (cameraPose != null) {
                telemetry.addData("Camera x (in)", "%.2f", cameraPose.position.x);
                telemetry.addData("Camera y (in)", "%.2f", cameraPose.position.y);
                telemetry.addData("Tag ID", tagId);
                double dx = cameraPose.position.x - pinpointPose.position.x;
                double dy = cameraPose.position.y - pinpointPose.position.y;
                telemetry.addData("Δ x (cam - pin)", "%.2f", dx);
                telemetry.addData("Δ y (cam - pin)", "%.2f", dy);
            } else {
                telemetry.addData("Camera", "No blue goal tag (ID %d)", CameraRelocalization.BLUE_GOAL_TAG_ID);
            }
            telemetry.update();

            // --- FTC Dashboard field overlay ---
            TelemetryPacket packet = new TelemetryPacket();
            Drawing.drawRobot(packet.fieldOverlay(), pinpointPose, 2, "#3F51B5");  // blue = Pinpoint
            if (cameraPose != null) {
                Drawing.drawCameraPose(packet.fieldOverlay(), cameraPose, "#FF5722");  // orange = camera
            }
            packet.put("pinpoint_x", pinpointPose.position.x);
            packet.put("pinpoint_y", pinpointPose.position.y);
            packet.put("pinpoint_heading_deg", Math.toDegrees(headingRad));
            if (cameraPose != null) {
                packet.put("camera_x", cameraPose.position.x);
                packet.put("camera_y", cameraPose.position.y);
                packet.put("tag_id", tagId);
            } else {
                packet.put("camera", "no tag");
            }
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            sleep(20);
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(909.963833736, 909.963833736, 634.495942075, 347.541434786)
                .build();
        aprilTag.setDecimation(3);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.setCameraResolution(new Size(1280, 720));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }
}
