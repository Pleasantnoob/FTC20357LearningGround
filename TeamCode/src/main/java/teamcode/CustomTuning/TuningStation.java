package teamcode.CustomTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import teamcode.ComplexMath.ShotRegression;
import teamcode.ComplexMath.Storage.PoseStorage;
import teamcode.OpModes.TeleOp.MainDrive;
import teamcode.RobotContainer.Robot;
import teamcode.Subsystems.Turret;
import teamcode.Utility.DashboardField;

/**
 * Tune everything from FTC Dashboard (http://192.168.43.1:8080/dash).
 *
 * DASHBOARD CONFIG GROUPS
 *   TuningStation .. flywheel on/off, target RPM, hood, turret fixed angle, raw servo tester
 *   Shooter ........ kV / kS / kP (flywheel PIDF)
 *   ShotRegression . your Desmos equations (rpm and hood vs distance)
 *   Turret ......... servo range, center, latency, pivot offset, targets...
 *   TurretGate ..... open / close      MatchLoader ... stowed / deployed
 *   TurretHood ..... min / max / default
 *   DashboardField . drawing rotation
 *
 * FLYWHEEL PIDF (graph "rpm" vs "target rpm")
 *   1. kP = 0, kV = 0. Raise kS until the flywheel just starts turning.
 *   2. Raise kV until "rpm" settles close to "target rpm" by itself.
 *   3. Raise kP until it recovers fast after a shot without oscillating.
 *
 * SERVO POSITIONS
 *   RAW_SERVO_ENABLE = true, RAW_SERVO_NAME = config name, slide RAW_SERVO_POS. Copy the numbers
 *   into TurretGate / MatchLoader / TurretHood. Turret servos are blocked here (geared together);
 *   use TURRET_AUTO_AIM = false + TURRET_FIXED_DEG instead.
 *
 * SHOT DATA FOR DESMOS
 *   Drive to a spot, adjust TARGET_RPM and HOOD_POS until shots score, write down
 *   "distance to target", RPM and hood. Fit in Desmos, type the coefficients into ShotRegression,
 *   then set USE_REGRESSION = true to check them.
 *
 * GAMEPAD 1: sticks drive | LT shoot (opens gate + feeds, no RPM check) | RT collect | RB eject
 *            LB toggle match loader
 * GAMEPAD 2: same turret / trim / alliance / pose-reset buttons as MainDrive,
 *            right bumper toggles shoot-on-the-move
 *
 * SHOOT ON THE MOVE (group "Turret": SHOOT_ON_THE_MOVE, FEED_TIME_SEC, TOF_A, TOF_B)
 *   Graph "vel x pinpoint" vs "vel x pose-diff" while driving: they should match. If the Pinpoint
 *   line looks wrong (flipped / robot-relative), set USE_PINPOINT_VELOCITY = false.
 */
@Config
@TeleOp(name = "TuningStation", group = "Tuning")
public class TuningStation extends LinearOpMode {

    // ---- Flywheel ----
    public static boolean FLYWHEEL_ON = false;
    public static double TARGET_RPM = 2240;
    /** true = RPM and hood come from the ShotRegression equations instead of the two values here. */
    public static boolean USE_REGRESSION = false;
    public static double READY_RPM_TOLERANCE = 40;

    // ---- Hood (clipped to TurretHood min/max) ----
    public static double HOOD_POS = 0.5;

    // ---- Turret ----
    public static boolean TURRET_AUTO_AIM = true;
    public static double TURRET_FIXED_DEG = 0;

    // ---- Raw servo tester ----
    public static boolean RAW_SERVO_ENABLE = false;
    public static String RAW_SERVO_NAME = "turretGate";
    public static double RAW_SERVO_POS = 0.5;

    private final PoseStorage poseStorage = new PoseStorage();
    private boolean pLB, p2Back, p2X, p2Y, p2DL, p2DR, p2LB, p2RB;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        if (poseStorage.getAlliance() == null || poseStorage.getAlliance() == PoseStorage.Alliance.NONE) {
            poseStorage.setAlliance(PoseStorage.Alliance.RED);
        }
        Follower follower = teamcode.pedro.Constants.create(hardwareMap);
        follower.setPose(new Pose(poseStorage.getPosX(), poseStorage.getPosY(), poseStorage.getRobotHeading()));

        String rawName = "";
        Servo rawServo = null;

        telemetry.addLine("TuningStation ready. Open FTC Dashboard.");
        telemetry.update();
        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        try {
            while (opModeIsActive()) {
                double dt = loopTimer.seconds();
                loopTimer.reset();

                // ---------- gamepad 2: alliance, trim, pose reset ----------
                if (pressed(gamepad2.x, p2X)) poseStorage.setAlliance(PoseStorage.Alliance.BLUE);
                if (pressed(gamepad2.y, p2Y)) poseStorage.setAlliance(PoseStorage.Alliance.RED);
                if (pressed(gamepad2.dpad_left, p2DL)) robot.turret.nudgeTrim(-1);
                if (pressed(gamepad2.dpad_right, p2DR)) robot.turret.nudgeTrim(+1);
                if (pressed(gamepad2.left_bumper, p2LB)) robot.turret.resetTrim();
                if (pressed(gamepad2.right_bumper, p2RB)) robot.turret.toggleShootOnTheMove();
                p2RB = gamepad2.right_bumper;
                if (pressed(gamepad2.back, p2Back)) {
                    boolean blue = poseStorage.getAlliance() == PoseStorage.Alliance.BLUE;
                    follower.setPose(blue
                            ? new Pose(MainDrive.BLUE_RESET_X, MainDrive.BLUE_RESET_Y, Math.toRadians(MainDrive.BLUE_RESET_HEADING_DEG))
                            : new Pose(MainDrive.RED_RESET_X, MainDrive.RED_RESET_Y, Math.toRadians(MainDrive.RED_RESET_HEADING_DEG)));
                }
                p2X = gamepad2.x;
                p2Y = gamepad2.y;
                p2DL = gamepad2.dpad_left;
                p2DR = gamepad2.dpad_right;
                p2LB = gamepad2.left_bumper;
                p2Back = gamepad2.back;

                // ---------- pose ----------
                follower.update();
                Pose pose = follower.pose();
                robot.turret.setMeasuredVelocity(follower.velocity().vx, follower.velocity().vy, follower.velocity().omega);
                double x = pose.x(), y = pose.y(), h = pose.heading();
                poseStorage.savePoseToStorage(x, y, h);

                // ---------- raw servo tester ----------
                boolean raw = RAW_SERVO_ENABLE && RAW_SERVO_NAME != null && !RAW_SERVO_NAME.isEmpty();
                String rawMsg = "off";
                if (raw && (RAW_SERVO_NAME.equals(Turret.LEFT_SERVO_NAME) || RAW_SERVO_NAME.equals(Turret.RIGHT_SERVO_NAME))) {
                    raw = false;
                    rawMsg = "BLOCKED: turret servos are geared together, use TURRET_FIXED_DEG";
                }
                if (raw) {
                    if (!RAW_SERVO_NAME.equals(rawName)) {
                        rawServo = hardwareMap.tryGet(Servo.class, RAW_SERVO_NAME);
                        rawName = RAW_SERVO_NAME;
                    }
                    if (rawServo != null) {
                        rawServo.setPosition(RAW_SERVO_POS);
                        rawMsg = RAW_SERVO_NAME + " = " + RAW_SERVO_POS;
                    } else {
                        rawMsg = "no servo named '" + RAW_SERVO_NAME + "' in the config";
                        raw = false;
                    }
                }
                boolean rawGate = raw && RAW_SERVO_NAME.equals("turretGate");
                boolean rawLoader = raw && RAW_SERVO_NAME.equals("matchLoader");
                boolean rawHood = raw && RAW_SERVO_NAME.equals("turretHood");

                // ---------- turret ----------
                if (TURRET_AUTO_AIM) {
                    robot.turret.update(x, y, h, gamepad2, dt);
                } else {
                    robot.turret.updateTurretTarget(y);
                    robot.turret.setAngleDegrees(TURRET_FIXED_DEG);
                }
                // effective (virtual-goal) distance when auto-aiming, real distance otherwise
                double distance = TURRET_AUTO_AIM ? robot.turret.getDistanceToTarget()
                        : robot.turret.distanceFromPivot(x, y, h);

                // ---------- RPM / hood source ----------
                double rpm = TARGET_RPM;
                double hood = HOOD_POS;
                if (USE_REGRESSION) {
                    rpm = ShotRegression.rpmFor(distance);
                    double eqHood = ShotRegression.hoodFor(distance);
                    if (!Double.isNaN(eqHood)) hood = eqHood;
                }

                // ---------- flywheel ----------
                boolean shoot = gamepad1.left_trigger > 0.1;
                robot.shooter.setBaseTargetRPM((FLYWHEEL_ON || shoot) ? (int) Math.round(rpm) : 0);
                robot.shooter.calculateShooterPID();
                double rpmNow = robot.shooter.getRPM();
                boolean atSpeed = robot.shooter.getTargetRPM() > 0
                        && Math.abs(robot.shooter.getTargetRPM() - rpmNow) < READY_RPM_TOLERANCE;

                // ---------- hood ----------
                if (!rawHood) robot.turretHood.setHoodAngle(hood);

                // ---------- intake / gate / loader ----------
                if (shoot) {
                    if (!rawGate) robot.turretGate.moveGateToOpenPos();
                    robot.intake.collect();
                } else {
                    if (!rawGate) robot.turretGate.moveGateToClosePos();
                    if (gamepad1.right_bumper) robot.intake.eject();
                    else if (gamepad1.right_trigger > 0.1) robot.intake.collect();
                    else robot.intake.stop();
                }
                if (pressed(gamepad1.left_bumper, pLB) && !rawLoader) robot.matchLoader.toggle();
                pLB = gamepad1.left_bumper;

                // ---------- drive ----------
                robot.drivetrain.updateDriveMotorPowers(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

                // ---------- dashboard: graphs + field ----------
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("rpm", rpmNow);
                packet.put("target rpm", robot.shooter.getTargetRPM());
                packet.put("rpm error", robot.shooter.getTargetRPM() - rpmNow);
                packet.put("flywheel power", robot.shooter.getLastPower());
                packet.put("distance to target", distance);
                packet.put("hood pos", robot.turretHood.getTargetPos());
                packet.put("turret cmd deg", robot.turret.getTurretHeading());
                packet.put("turret actual deg", robot.turret.getActualDeg());
                packet.put("turret on target", robot.turret.isOnTarget());
                packet.put("at speed", atSpeed);
                packet.put("x", x);
                packet.put("y", y);
                packet.put("heading deg", Math.toDegrees(h));
                packet.put("loop ms", dt * 1000);
                packet.put("shoot on the move", robot.turret.isShootOnTheMove());
                packet.put("vel x pinpoint", robot.turret.getVelX());
                packet.put("vel x pose-diff", robot.turret.getDiffVelX());
                packet.put("vel y pinpoint", robot.turret.getVelY());
                packet.put("vel y pose-diff", robot.turret.getDiffVelY());
                packet.put("omega pinpoint", robot.turret.getOmega());
                packet.put("omega pose-diff", robot.turret.getDiffOmega());
                DashboardField.draw(packet.fieldOverlay(), x, y, h, robot.turret);
                dashboard.sendTelemetryPacket(packet);

                // ---------- driver station ----------
                telemetry.addData("READY", atSpeed && robot.turret.isOnTarget() ? "YES - shoot" : "no");
                telemetry.addData("distance", "%.1f in", distance);
                telemetry.addData("rpm", "%.0f / %d", rpmNow, robot.shooter.getTargetRPM());
                telemetry.addData("hood", "%.3f", robot.turretHood.getTargetPos());
                telemetry.addData("turret", "%.1f deg (%s)", robot.turret.getTurretHeading(), robot.turret.getAimMode());
                telemetry.addData("alliance", poseStorage.getAlliance());
                telemetry.addData("shoot on the move", robot.turret.isShootOnTheMove() ? "ON" : "off");
                telemetry.addData("source", USE_REGRESSION ? "ShotRegression equations" : "TARGET_RPM / HOOD_POS");
                telemetry.addData("raw servo", rawMsg);
                telemetry.update();
            }
        } finally {
            robot.stop();
        }
    }

    private static boolean pressed(boolean now, boolean before) {
        return now && !before;
    }
}
