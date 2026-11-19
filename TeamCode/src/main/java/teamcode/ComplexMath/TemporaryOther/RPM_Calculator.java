package teamcode.ComplexMath.TemporaryOther;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Not used. Early placeholder for the RPM/hood regression; ShotRegression does this now.
 * (Its hardwareMap import is the Blocks one, so it wouldn't work if called.)
 */
public class RPM_Calculator {

    DcMotorEx flywheelMotor;
    public RPM_Calculator() {

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");



    }



    public double IdealRPMRegression(){
        //put rpm regression formula here (use distance(in) and RPM for desmos)
        /*
            likely won't use Range.clip for this one. It is unlikely we will need to limit the flywheel output since
            the max is 6000RPM for the shooter motor, but should keep
         */
        return 0;

    }

    public double idealHoodAngleRegression(){
        //put hood angle formula here (use distance(in) and hood angle for desmos)
        // will probably use Range.clip for this one for if the regression tries to get the position to go beyond 1.0
        return 0;
    }




}
