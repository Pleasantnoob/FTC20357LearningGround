package teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;



public class Shooter {

    //CONSTANTS
        //put in actual shooterAngle when designers tells me
        private final double shooterAngle = 25;


        //fill in with actual height relative to camera unless I find a way to do it from shooter
        //believe I put a heightThreshold variable in the vision class. can use that to define h.

        //
        // plugging in 43 for now. It is the number I measured from the ground to a midpoint between
        // the displacement of the top and bottom cell; need to measure the distance from the camera to the ground and subtract it from this
        // because the camera measures the apriltag's height from the camera, not the ground
        private final double h = 43;

    //PID Tuning Values

        // Feedforward was tuned when the reported RPM was 4x too small, then the
        // target moved 560 -> 2240 without scaling the gains. 0.0002*2240+0.07
        // is about 0.52, which is the old physical power at the corrected RPM.
        public static double kV = 0.0002;
        public static double kS = 0.07;
        public static double kP = 0.00125;

    //Objects
        //put tuning constants in class parameters when I make the PIDController class
        private DcMotorEx shooterMotor;
        //private DcMotorEx rightShooterMotor;
        public int baseTargetRPM = 0;
        private double lastPower = 0;

    public Shooter(HardwareMap hardwareMap) {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");

            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterMotor.setPower(0);
    }




        public double getTicksPerSec(){
            //Check if this is the right Vel
            return shooterMotor.getVelocity();

        }

        public void setShooterMotorPowers(double power){
            double clipped = Range.clip(power, -1, 1);
            if (clipped == lastPower) {
                return;
            }
            lastPower = clipped;
            shooterMotor.setPower(clipped);
        }

        public void emergencyShooterStop(){
            baseTargetRPM = 0;
            lastPower = 0;
            shooterMotor.setPower(0);
        }


        public double getRPM(){
            // goBILDA 6000 RPM motor on the flywheel. 28 counts per motor revolution.
            // The 435 RPM motor (384.5 counts) is the turret, not this shooter.
            double gearRatio = 1;
            double ticksPerRev = 28;

            double TICKS_PER_FLYWHEEL_REVOLUTION = ticksPerRev * gearRatio;
            return (getTicksPerSec()/ TICKS_PER_FLYWHEEL_REVOLUTION) * 60;

        }

        public void calculateShooterPID(){
            if (baseTargetRPM == 0) {
                setShooterMotorPowers(0);
                return;
            }

            double feedForward = (kV * baseTargetRPM) + kS;
            double error = baseTargetRPM - getRPM();
            double feedBack = error * kP;
            setShooterMotorPowers(Range.clip(feedForward + feedBack, -1, 1));
        }



    // --------------
    // Basic Shooting
    // --------------


    public void setBaseTargetRPM(int targRPMBase){
        baseTargetRPM = targRPMBase;
    }



    public int getTargetRPM(){
        return baseTargetRPM;

    }













}
