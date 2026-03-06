package org.firstinspires.ftc.teamcode.RR.tuning;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.RR.PinpointLocalizer;

/**
 * Push test: display raw odometry ticks while you push the robot.
 * Use to calculate inPerTick (par and perp use same value).
 *
 * 1. Run this opmode.
 * 2. Press A to reset tick counters.
 * 3. Push robot FORWARD a known distance (e.g. 24 inches). Note par ticks.
 * 4. Press A to reset.
 * 5. Push robot LATERALLY (strafe) a known distance (e.g. 24 inches). Note perp ticks.
 *
 * inPerTick = inches pushed / ticks (use par or perp; both should match)
 * Example: 24 inches, 12300 ticks → inPerTick = 24/12300 ≈ 0.00195
 */
@TeleOp(name = "Odom Ticks Test", group = "tuning")
public class OdomTicksTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GoBildaPinpointDriver driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        double inPerTick = MecanumDrive.PARAMS.inPerTick;
        double mmPerTick = inPerTick * 25.4;
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        driver.setOffsets(
                mmPerTick * PinpointLocalizer.PARAMS.parYTicks,
                mmPerTick * PinpointLocalizer.PARAMS.perpXTicks,
                DistanceUnit.MM);
        driver.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        driver.resetPosAndIMU();

        int parAtReset = 0;
        int perpAtReset = 0;
        boolean prevA = false;

        telemetry.clear();
        telemetry.addLine("Odom Ticks Test - push robot to count ticks");
        telemetry.addLine("Press A to reset counters");
        telemetry.addLine("---");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            int parTicks = 0, perpTicks = 0;
            String err = null;
            try {
                driver.update();
                if (gamepad1.a && !prevA) {
                    parAtReset = driver.getEncoderX();
                    perpAtReset = driver.getEncoderY();
                }
                prevA = gamepad1.a;
                parTicks = driver.getEncoderX();
                perpTicks = driver.getEncoderY();
            } catch (Exception e) {
                err = e.getMessage();
            }
            int parDelta = parTicks - parAtReset;
            int perpDelta = perpTicks - perpAtReset;

            telemetry.clear();
            telemetry.addLine("Odom Ticks Test - push robot");
            telemetry.addLine("Press A to reset");
            telemetry.addLine("---");
            if (err != null) {
                telemetry.addLine("ERROR: " + err);
            } else {
                telemetry.addData("Par (forward) ticks", parTicks);
                telemetry.addData("Perp (lateral) ticks", perpTicks);
                telemetry.addLine("---");
                telemetry.addData("Par delta (since reset)", parDelta);
                telemetry.addData("Perp delta (since reset)", perpDelta);
            }
            telemetry.addLine("---");
            telemetry.addData("inPerTick", "inches / ticks (par or perp)");
            telemetry.update();

            sleep(20);
        }
    }
}
