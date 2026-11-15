package teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    // Both rollers are 1620 RPM motors run open-loop. Tick counts are unused.
    private final DcMotor intakeMotor;
    private final DcMotor transferMotor;
    private double intakePower = 0;
    private double transferPower = 0;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setPower(0);
        transferMotor.setPower(0);
    }

    public void collect() {
        setIntakePower(1);
        setTransferPower(1);
    }

    public void eject() {
        setIntakePower(-1);
        setTransferPower(-1);
    }

    public void stop() {
        setIntakePower(0);
        setTransferPower(0);
    }

    public void forceStop() {
        intakePower = 0;
        transferPower = 0;
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
    }

    public void justFrontIntake() {
        setIntakePower(1);
    }

    public void justFrontEject() {
        setIntakePower(-1);
    }

    public void justTransIntake() {
        setTransferPower(1);
    }

    public void justTransEject() {
        setTransferPower(-1);
    }

    public void fullIntake() {
        collect();
    }

    public void fullEject() {
        eject();
    }

    public void zeroTransfer() {
        setTransferPower(0);
    }

    public void zeroIntake() {
        setIntakePower(0);
    }

    private void setIntakePower(double power) {
        if (power == intakePower) {
            return;
        }
        intakePower = power;
        intakeMotor.setPower(power);
    }

    private void setTransferPower(double power) {
        if (power == transferPower) {
            return;
        }
        transferPower = power;
        transferMotor.setPower(power);
    }
}
