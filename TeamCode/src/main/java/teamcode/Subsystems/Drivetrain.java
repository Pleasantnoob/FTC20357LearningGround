package teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Mecanum drive for TeleOp. Plain open-loop power from the sticks, no PID.
 * Autos don't use this class; Pedro drives the same four motors directly.
 * Motor names must match the Driver Station config: lf_motor, lb_motor, rf_motor, rb_motor.
 */
public class Drivetrain {

    // Teleop is open-loop power. Belted output is about 500 RPM, which does not
    // change this mix. Pedro's max velocity still needs a retune on the new drive.

    private DcMotor lf_motor;
    private DcMotor lb_motor;
    private DcMotor rf_motor;
    private DcMotor rb_motor;
    private double absTotalPower;
    private double lf_motor_power;
    private double lb_motor_power;
    private double rf_motor_power;
    private double rb_motor_power;
    private boolean stopped = true;

    public Drivetrain(HardwareMap hardwareMap){
        lf_motor = hardwareMap.get(DcMotor.class, "lf_motor");
        lb_motor = hardwareMap.get(DcMotor.class, "lb_motor");
        rf_motor = hardwareMap.get(DcMotor.class, "rf_motor");
        rb_motor = hardwareMap.get(DcMotor.class, "rb_motor");

        // Left side is reversed so +power drives every wheel forward. Keep this matching
        // the directions in pedro/Constants.java or TeleOp and autos will disagree.
        lf_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        lb_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rf_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        rb_motor.setDirection(DcMotorSimple.Direction.FORWARD);



        lf_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // BRAKE so the robot stops when the sticks are let go instead of coasting.
        lf_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cutPower();
    }


    /**
     * Standard mecanum mix.
     * @param vertical   left stick Y (up on the stick is negative, so it gets flipped below)
     * @param horizontal turn, right stick X
     * @param strafe     left stick X
     */
    public void updateDriveMotorPowers(double vertical, double horizontal, double strafe){
        // If the inputs add up past 1, scale everything down together so the robot
        // keeps driving in the direction you pushed instead of clipping one wheel.

        absTotalPower = Math.max((Math.abs(vertical) + Math.abs(horizontal) + Math.abs(strafe)), 1);
        lf_motor_power = -vertical + horizontal + strafe;
        lb_motor_power = -vertical + horizontal - strafe;
        rf_motor_power = -vertical - horizontal - strafe;
        rb_motor_power = -vertical - horizontal + strafe;

        // Only send new powers while the sticks are moving; once they're centered, zero the
        // motors one time and then stop writing (saves hub calls every loop).
        if((Math.abs(vertical) + Math.abs(horizontal) + Math.abs(strafe)) != 0) {
            stopped = false;
            lf_motor.setPower(lf_motor_power / absTotalPower);
            lb_motor.setPower(lb_motor_power / absTotalPower);
            rf_motor.setPower(rf_motor_power / absTotalPower);
            rb_motor.setPower(rb_motor_power / absTotalPower);

        } else if (!stopped) {
            cutPower();
        }
    }

    public void stop() {
        cutPower();
    }

    private void cutPower() {
        lf_motor.setPower(0);
        lb_motor.setPower(0);
        rf_motor.setPower(0);
        rb_motor.setPower(0);
        lf_motor_power = 0;
        lb_motor_power = 0;
        rf_motor_power = 0;
        rb_motor_power = 0;
        stopped = true;
    }









    // Last powers sent, for telemetry.
    public double getLFMotorPower(){
        return lf_motor_power;
    }
    public double getLBMotorPower(){
        return lb_motor_power;
    }
    public double getRFMotorPower(){
        return rf_motor_power;
    }
    public double getRBMotorPower(){
        return rb_motor_power;
    }

















}
