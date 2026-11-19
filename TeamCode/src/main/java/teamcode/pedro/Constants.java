package teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    // Pedro Pathing setup. Pedro uses these for autos and for tracking position in TeleOp.

    // Drive motor names and directions. Must match Drivetrain.java.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("lf_motor");
        c.frontRightName.set("rf_motor");
        c.backLeftName.set("lb_motor");
        c.backRightName.set("rb_motor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });



    // MECANUM CONFIG IS DONE, BUT PINPOINT CONFIG AND FORESIGHT CONFIG STILL NEED TO BE COMPLETED



// Pinpoint odometry. Pod offsets are inches from the robot's center of rotation; that center
// is what Pedro calls "the robot's position". These are from the OLD robot - rerun the Pinpoint
// tuner (web page at 192.168.43.1:10158) on V2 and paste the new offsets/directions here.
public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
    c.name.set("pinpoint");
    c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    c.xPodOffset.set(-6.310756413016733);
    c.yPodOffset.set(1.6268964452067698);
    c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    c.globalDistanceUnit.set(DistanceUnit.INCH);
    c.offsetUnits.set(DistanceUnit.INCH);
});






    // Path-following gains, max speeds and braking. All of this comes from the Foresight tuner.
    // Old drivetrain numbers - rerun the tuner after the Pinpoint tuner on the new 500 rpm drive.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2880526535156293);
                Controller secondaryTranslationalForward = Controller.proportional(0.1064277585653671);
                Controller primaryTranslationalLateral = Controller.proportional(0.37745472971797234);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1394594368546603);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01807957638095843));
                c.brake.set(Controller.proportionalFeedforward(0.015367639923814666));

                c.headingFeedback.set(Controller.proportional(3.4634823337064455));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04702930488656839, 0.004068184896825027));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06575157481259479, 0.12348389871086035));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0017839531992387408, -0.0012276570472928047));

                c.maxAchievableForwardVelocity.set(57.558379764614266);
                c.maxAchievableStrafeVelocity.set(47.855203063797475);
                c.naturalForwardDeceleration.set(37.82139773130478);
                c.naturalStrafeDeceleration.set(67.24941856418013);
            }
    );

    /** Makes a follower from the configs above. Every OpMode that needs position calls this. */
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
