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
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

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
 * NOTHING MOVES UNTIL YOU TURN IT ON
 *   Every servo starts off: TURRET_MODE = OFF (both turret servos limp), HOOD_ON / GATE_ON /
 *   LOADER_ON = false, flywheel off. Turn each one on only after its positions are tuned.
 *
 * TURRET (group TuningStation: TURRET_MODE, TURRET_RAW_POS)
 *   OFF ........ both servos limp, turn the turret by hand
 *   LEFT_ONLY .. only the left servo gets a signal (TURRET_RAW_POS); the right one is limp
 *   RIGHT_ONLY . same for the right servo
 *   BOTH_RAW ... left = TURRET_RAW_POS, right = TURRET_RAW_POS + Turret.rightOffsetPos
 *   FIXED_DEG .. both servos through the normal angle math, at TURRET_FIXED_DEG
 *   AUTO_AIM ... normal aiming, same gamepad 2 controls as MainDrive
 *   Steps:
 *   1. LEFT_ONLY: find the position where the turret points straight forward -> Turret.centerPos.
 *      Go to both ends to find servoRangeDeg / travel. Check + turns it left (else invertDirection).
 *   2. RIGHT_ONLY: same center? If the turret turns the opposite way, reverseRight is wrong.
 *   3. BOTH_RAW at centerPos: nudge Turret.rightOffsetPos until "hub current A" is lowest and the
 *      servos stop buzzing. That's them no longer fighting.
 *   4. FIXED_DEG, then AUTO_AIM.
 *
 * OTHER SERVO POSITIONS
 *   RAW_SERVO_ENABLE = true, RAW_SERVO_NAME = config name, slide RAW_SERVO_POS. Copy the numbers
 *   into TurretGate / MatchLoader / TurretHood. Use this (not HOOD_POS) to find the hood limits,
 *   because HOOD_POS is clipped to TurretHood min/max. Turret servos are blocked here, use TURRET_MODE.
 *
 * SHOT DATA FOR DESMOS
 *   Drive to a spot, adjust TARGET_RPM and HOOD_POS until shots score, write down
 *   "distance to target", RPM and hood. Fit in Desmos, type the coefficients into ShotRegression,
 *   then set USE_REGRESSION = true to check them.
 *
 * GAMEPAD 1: sticks drive | LT shoot (flywheel + feed, gate opens if GATE_ON, no RPM check)
 *            RT collect | RB eject | LB toggle match loader (if LOADER_ON)
 * GAMEPAD 2: same turret / trim / alliance / pose-reset buttons as MainDrive (turret ones only
 *            in AUTO_AIM), right bumper toggles shoot-on-the-move
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
    /** false = the hood servo is never written (stays limp) until you've found its limits. */
    public static boolean HOOD_ON = false;
    public static double HOOD_POS = 0.5;

    // ---- Gate / match loader ----
    /** false = the gate servo is never written. */
    public static boolean GATE_ON = false;
    /** false = gamepad 1 LB does nothing to the match loader. */
    public static boolean LOADER_ON = false;

    // ---- Turret ----
    public enum TurretMode { OFF, LEFT_ONLY, RIGHT_ONLY, BOTH_RAW, FIXED_DEG, AUTO_AIM }
    public static TurretMode TURRET_MODE = TurretMode.OFF;
    /** Raw servo position for LEFT_ONLY / RIGHT_ONLY / BOTH_RAW. */
    public static double TURRET_RAW_POS = 0.5;
    public static double TURRET_FIXED_DEG = 0;

    // ---- Graphs ----
    /** Total current of all hubs. Fighting turret servos show up as a jump here. Costs a few ms per loop. */
    public static boolean SHOW_HUB_CURRENT = true;

    // ---- Raw servo tester ----
    public static boolean RAW_SERVO_ENABLE = false;
    public static String RAW_SERVO_NAME = "turretGate";
    public static double RAW_SERVO_POS = 0.5;

    private final PoseStorage poseStorage = new PoseStorage();
    private boolean pLB, p2Back, p2X, p2Y, p2DL, p2DR, p2LB, p2RB;

    @Override
    public void runOpMode() {
        // false = turret servos stay limp at init instead of jumping to center
        Robot robot = new Robot(hardwareMap, false);
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        TurretMode appliedMode = null;
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
                    rawMsg = "BLOCKED: turret servos are geared together, use TURRET_MODE";
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
                TurretMode mode = TURRET_MODE == null ? TurretMode.OFF : TURRET_MODE;
                if (mode != appliedMode) {
                    // switch servo signals only when the mode changes
                    boolean left = mode != TurretMode.OFF && mode != TurretMode.RIGHT_ONLY;
                    boolean right = mode != TurretMode.OFF && mode != TurretMode.LEFT_ONLY;
                    robot.turret.setPowered(left, right);
                    appliedMode = mode;
                }
                switch (mode) {
                    case LEFT_ONLY:
                    case RIGHT_ONLY:
                        robot.turret.writeRaw(TURRET_RAW_POS, TURRET_RAW_POS); // the limp one is skipped
                        break;
                    case BOTH_RAW:
                        robot.turret.writeRaw(TURRET_RAW_POS, TURRET_RAW_POS + Turret.rightOffsetPos);
                        break;
                    case FIXED_DEG:
                        robot.turret.updateTurretTarget(y);
                        robot.turret.setAngleDegrees(TURRET_FIXED_DEG);
                        break;
                    case AUTO_AIM:
                        robot.turret.update(x, y, h, gamepad2, dt);
                        break;
                    default: // OFF: nothing is written
                        robot.turret.updateTurretTarget(y);
                        break;
                }
                // effective (virtual-goal) distance when auto-aiming, real distance otherwise
                double distance = mode == TurretMode.AUTO_AIM ? robot.turret.getDistanceToTarget()
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
                if (HOOD_ON && !rawHood) robot.turretHood.setHoodAngle(hood);

                // ---------- intake / gate / loader ----------
                if (shoot) {
                    if (GATE_ON && !rawGate) robot.turretGate.moveGateToOpenPos();
                    robot.intake.collect();
                } else {
                    if (GATE_ON && !rawGate) robot.turretGate.moveGateToClosePos();
                    if (gamepad1.right_bumper) robot.intake.eject();
                    else if (gamepad1.right_trigger > 0.1) robot.intake.collect();
                    else robot.intake.stop();
                }
                if (pressed(gamepad1.left_bumper, pLB) && LOADER_ON && !rawLoader) robot.matchLoader.toggle();
                pLB = gamepad1.left_bumper;

                // ---------- drive ----------
                robot.drivetrain.updateDriveMotorPowers(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

                // ---------- dashboard: graphs + field ----------
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("rpm", rpmNow);
                packet.put("target rpm", robot.shooter.getTargetRPM());
                packet.put("rpm error", robot.shooter.getTargetRPM() - rpmNow);
                packet.put("flywheel power", robot.shooter.getLastPower());
                packet.put("battery V", robot.shooter.getBatteryVoltage());
                packet.put("distance to target", distance);
                packet.put("hood pos", robot.turretHood.getTargetPos());
                packet.put("turret cmd deg", robot.turret.getTurretHeading());
                packet.put("turret actual deg", robot.turret.getActualDeg());
                packet.put("turret on target", robot.turret.isOnTarget());
                if (SHOW_HUB_CURRENT) {
                    double amps = 0;
                    for (LynxModule hub : hubs) amps += hub.getCurrent(CurrentUnit.AMPS);
                    packet.put("hub current A", amps);
                }
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
                telemetry.addData("turret mode", mode);
                telemetry.addData("hood / gate / loader", "%s / %s / %s",
                        HOOD_ON ? "on" : "OFF", GATE_ON ? "on" : "OFF", LOADER_ON ? "on" : "OFF");
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
