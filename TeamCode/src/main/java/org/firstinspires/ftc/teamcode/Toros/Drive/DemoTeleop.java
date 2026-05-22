package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Toros.Autonomous.LedFadeAction;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.IntakeV2;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Pose2d;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * Showcase teleop: drive disabled, all controls on Gamepad 2.
 *
 * <p><b>G2 controls</b>
 * <ul>
 *   <li>Left stick X — manual turret (robot-relative)</li>
 *   <li>D-pad Up/Down — target velocity ±100</li>
 *   <li>D-pad Left/Right — hood angle ±2°</li>
 *   <li>Left trigger — rev flywheel</li>
 *   <li>Right trigger — shoot when at speed</li>
 *   <li>A — center turret</li>
 *   <li>X / Start — turret encoder re-sync</li>
 *   <li>Right bumper — intake in</li>
 *   <li>Left bumper — intake out</li>
 *   <li>B — hard stop (flywheel, intake, transfer)</li>
 * </ul>
 */
@TeleOp(name = "Demo / Showcase", group = "Toros")
@Config
public class DemoTeleop extends LinearOpMode {
    private static final int LOOP_SLEEP_MS = 20;
    private static final double MANUAL_DEG_PER_UNIT = 4.0;
    private static final int RESYNC_RUMBLE_MS = 300;

    public static double startX = -50.0;
    public static double startY = -50.0;
    public static double startHeadingDeg = -128.0;

    private Servo led;
    private PedroDrive pedroDrive;
    private IntakeV2 intake;
    private Turret turret;

    private double k;
    private double manualTurretDeg;
    private boolean prevStart;
    private boolean turretInitialized;
    private final ElapsedTime ledTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        led = hardwareMap.get(Servo.class, "LED");
        IntakeV2.manualMode = true;
        IntakeV2.manualTargetVel = -1600.0;
        IntakeV2.manualHoodAngleDeg = 55.0;
        IntakeV2.demoTargetVel = -1600.0;

        intake = new IntakeV2(hardwareMap, null, gamepad2, null);
        turret = new Turret(hardwareMap, gamepad2, false);
        pedroDrive = new PedroDrive(hardwareMap, new Pose2d(startX, startY, Math.toRadians(startHeadingDeg)));

        while (!isStarted() && opModeIsActive()) {
            telemetry.addLine("--- Demo / Showcase (Gamepad 2 only) ---");
            telemetry.addLine("Stick X = turret | D-pad U/D = vel ±100 | D-pad L/R = hood ±2°");
            telemetry.addLine("LT = rev | RT = fire at speed | A = center | X/Start = resync");
            telemetry.addLine("RB = intake in | LB = intake out | B = hard stop");
            telemetry.addData("Target vel", IntakeV2.demoTargetVel);
            telemetry.addData("Hood deg", IntakeV2.manualHoodAngleDeg);
            telemetry.update();
            sleep(LOOP_SLEEP_MS);
        }

        waitForStart();
        ledTimer.reset();

        while (opModeIsActive()) {
            pedroDrive.update();
            pedroDrive.setTeleOpDrive(0, 0, 0, false);

            Pose2d pose = pedroDrive.getPose();
            turret.botHeading = Turret.wrapDeg360(Math.toDegrees(pose.heading));

            if (!turretInitialized) {
                k = turret.botHeading;
                manualTurretDeg = turret.getTurretAngleRobot();
                turretInitialized = true;
            }

            led.setPosition(LedFadeAction.positionAtSeconds(ledTimer.seconds()));

            if (gamepad2.xWasPressed() || (gamepad2.start && !prevStart)) {
                turret.resyncEncoder();
                manualTurretDeg = 0;
                turret.targetAngle = k;
                gamepad2.rumble(RESYNC_RUMBLE_MS);
            }
            prevStart = gamepad2.start;

            updateManualTurret();
            turret.runTurretNoGyro(k);

            intake.runLauncherDemo();
            intake.runDemoIntake();

            updateTelemetry();
            sleep(LOOP_SLEEP_MS);
        }
    }

    private void updateManualTurret() {
        if (gamepad2.aWasPressed()) {
            manualTurretDeg = 0;
        }
        manualTurretDeg += gamepad2.left_stick_x * MANUAL_DEG_PER_UNIT;
        manualTurretDeg = Math.max(-Turret.manualModeLimitDeg, Math.min(Turret.manualModeLimitDeg, manualTurretDeg));
        turret.setAngle(k + manualTurretDeg);
    }

    private void updateTelemetry() {
        double actualVel = intake.launch.getVelocity();
        double targetVel = intake.getDemoTargetVel();
        double velError = Math.abs(actualVel - targetVel);
        boolean atSpeed = intake.isAtTargetSpeed();

        telemetry.addLine("--- Demo / Showcase (G2 only) ---");
        telemetry.addData("Drive", "DISABLED");
        telemetry.addData("Turret robot deg", turret.getTurretAngleRobot());
        telemetry.addData("Turret target field deg", turret.targetAngle);
        telemetry.addLine("");
        telemetry.addLine("--- Launcher ---");
        telemetry.addData("Target vel", "%.0f", targetVel);
        telemetry.addData("Actual vel", "%.0f", actualVel);
        telemetry.addData("Vel error", "%.0f (tol ±%d)", velError, IntakeV2.SHOOT_VEL_TOLERANCE);
        telemetry.addData("AT SPEED", atSpeed && gamepad2.left_trigger > 0.1);
        telemetry.addData("Hood deg", IntakeV2.manualHoodAngleDeg);
        telemetry.addLine("");
        telemetry.addLine("--- Intake ---");
        telemetry.addData("RB", "intake in | LB = out | B = hard stop");
        telemetry.update();
    }
}
