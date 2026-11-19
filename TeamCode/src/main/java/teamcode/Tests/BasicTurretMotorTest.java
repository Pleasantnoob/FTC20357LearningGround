package teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import teamcode.Subsystems.Turret;

/**
 * (Hidden) Moves the turret by hand with gamepad 1 right stick. Quick check that both turret servos move together.
 */
@Disabled
@TeleOp
public class BasicTurretMotorTest extends LinearOpMode {

    public void runOpMode() {
        Turret turret = new Turret(hardwareMap);

        waitForStart();
        ElapsedTime loopTimer = new ElapsedTime();
        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            double stick = gamepad1.right_stick_x;
            if (Math.abs(stick) > Turret.stickDeadband) {
                turret.slew(stick, dt);
            }

            telemetry.addData("commandedDeg", turret.getTurretHeading());
            telemetry.addData("leftServo", turret.getLeftServoPosition());
            telemetry.update();
        }
    }
}
