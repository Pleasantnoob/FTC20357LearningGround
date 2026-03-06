package org.firstinspires.ftc.teamcode.RR;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.RR.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.RR.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.RR.messages.PoseMessage;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class       MecanumDrive {
    public static class Params {
        // 2 dead wheel odometry: GoBilda pods. Both par and perp use same resolution for identical pods.
        /** Par and perp use same value. 4-Bar (32mm)≈0.00198, Swingarm (48mm)≈0.00290. Tunable on FTC Dashboard. */
        public double inPerTick = 0.00194723378259707;
        public double trackWidthTicks = 7890.2951262221695;

        // feedforward parameters (in tick units)
        public double kS = 0.8999998620954397;
        public double kV = 0.0003411155484269247;
        public double kA = 0.00006;

        // path profile parameters (in inches)
        public double maxWheelVel = 100;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 100;

        // turn profile parameters (in radians)
        public double maxAngVel =  Math.PI; // shared with path
        public double maxAngAccel =  Math.PI;

        // path controller gains
        public double axialGain = 14;
        public double lateralGain = 10;
        public double headingGain = 9; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn

        // Position hold (teleop idle): lower gains to avoid aggressive correction from odometry noise
        public double holdAxialGain = 3.0;
        public double holdLateralGain = 3.0;
        public double holdHeadingGain = 2.0;
        /** Position error deadband (in): below this, no correction. Reduces jitter from odometry noise. */
        public double holdPosDeadband = 0.4;
        /** Heading error deadband (rad): below this, no correction. */
        public double holdHeadingDeadband = 0.05;
    }

    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, 1.0);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public final Localizer localizer;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        leftBack = hardwareMap.get(DcMotorEx.class, "bl");
        rightBack = hardwareMap.get(DcMotorEx.class, "br");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);


        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);



        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new PinpointLocalizer(hardwareMap, PARAMS.inPerTick, pose);


        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    /**
     * Computes velocity command to hold position against external pushes.
     * Uses lower hold gains and error deadband to avoid aggressive correction from odometry noise.
     * Call when driver input is idle; apply via setDrivePowers().
     */
    public PoseVelocity2d computeHoldCorrection(Pose2d holdPose) {
        Pose2d actual = localizer.getPose();
        Pose2d error = holdPose.minusExp(actual);
        double ex = error.position.x;
        double ey = error.position.y;
        double eh = error.heading.toDouble();
        // Deadband: ignore small errors (odometry noise)
        double db = PARAMS.holdPosDeadband;
        if (Math.abs(ex) < db) ex = 0;
        if (Math.abs(ey) < db) ey = 0;
        if (Math.abs(eh) < PARAMS.holdHeadingDeadband) eh = 0;
        double vx = PARAMS.holdAxialGain * ex;
        double vy = PARAMS.holdLateralGain * ey;
        double vh = PARAMS.holdHeadingGain * eh;
        PoseVelocity2d cmd = new PoseVelocity2d(new Vector2d(vx, vy), vh);
        // Clamp magnitude to prevent runaway on odometry glitches
        final double maxLinVel = 50.0;
        final double maxAngVel = Math.PI;
        double linMag = Math.hypot(cmd.linearVel.x, cmd.linearVel.y);
        if (linMag > maxLinVel) {
            double scale = maxLinVel / linMag;
            cmd = new PoseVelocity2d(
                    new Vector2d(cmd.linearVel.x * scale, cmd.linearVel.y * scale),
                    cmd.angVel);
        }
        if (Math.abs(cmd.angVel) > maxAngVel) {
            cmd = new PoseVelocity2d(cmd.linearVel,
                    Math.signum(cmd.angVel) * maxAngVel);
        }
        return cmd;
    }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());
        
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }
        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));
        
        
        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
}
