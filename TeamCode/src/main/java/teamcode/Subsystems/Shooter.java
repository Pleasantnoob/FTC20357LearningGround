package teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;



/**
 * Flywheel (one 6000 rpm motor, config name "leftShooterMotor").
 * Set a target with setBaseTargetRPM(), then call calculateShooterPID() every loop.
 * Control is feedforward (kS + kV * target, scaled for battery voltage) plus a P term.
 * Gains are live-tunable in FTC Dashboard under "Shooter".
 */
@Config
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
        // power = (kS + kV * targetRPM) * (NOMINAL_VOLTAGE / battery) + kP * (targetRPM - rpm)
        //   kS = power to overcome friction, kV = power per RPM. Together they guess the power
        //   needed BEFORE any error appears; kP only cleans up what's left (mostly shot recovery).
        // Old-robot values - retune on the V2 flywheel (TuningStation).
        public static double kV = 0.0002;
        public static double kS = 0.07;
        public static double kP = 0.00125;

        /** Scales the feedforward by NOMINAL_VOLTAGE / battery so the RPM doesn't sag as the battery drains. */
        public static boolean VOLTAGE_COMP = true;
        /** Tune kS / kV with VOLTAGE_COMP on; they're then "power at this voltage". */
        public static double NOMINAL_VOLTAGE = 12.5;
        public static double VOLTAGE_READ_MS = 200;

    //Objects
        //put tuning constants in class parameters when I make the PIDController class
        private DcMotorEx shooterMotor;
        //private DcMotorEx rightShooterMotor;
        public int baseTargetRPM = 0;
        private double lastPower = 0;
        private final VoltageSensor batterySensor;
        private double batteryVolts = 12.5;
        private long lastVoltageReadMs = 0;

    public Shooter(HardwareMap hardwareMap) {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");

            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterMotor.setPower(0);

            batterySensor = hardwareMap.voltageSensor.iterator().hasNext()
                    ? hardwareMap.voltageSensor.iterator().next() : null;
    }

        /** Battery voltage, re-read every VOLTAGE_READ_MS (each read is a hub call, so not every loop). */
        public double getBatteryVoltage(){
            long now = System.currentTimeMillis();
            if (batterySensor != null && now - lastVoltageReadMs >= VOLTAGE_READ_MS) {
                double v = batterySensor.getVoltage();
                if (v > 6) batteryVolts = v; // ignore bad / brownout readings
                lastVoltageReadMs = now;
            }
            return batteryVolts;
        }




        public double getTicksPerSec(){
            //Check if this is the right Vel
            return shooterMotor.getVelocity();

        }

        // false = flywheel always gets 0 power (see RobotContainer/Tuned)
        private boolean enabled = true;
        public void setEnabled(boolean on) { enabled = on; if (!on) emergencyShooterStop(); }
        public boolean isEnabled() { return enabled; }

        public void setShooterMotorPowers(double power){
            double clipped = enabled ? Range.clip(power, -1, 1) : 0;
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


        public double getLastPower(){
            return lastPower;
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
            if (VOLTAGE_COMP) {
                feedForward *= NOMINAL_VOLTAGE / getBatteryVoltage();
            }
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
