package teamcode.Tests.AllianceStorageTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * (Hidden) Shows the alliance/pose saved by AllianceTestAuto.
 */
@Disabled
@TeleOp
public class AllianceTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode(){

        TestAllianceStorage testAllianceStorage = new TestAllianceStorage();

        waitForStart();

        while(opModeIsActive()){

            if(testAllianceStorage.getTestAlliance() == TestAllianceStorage.TestAlliance.TEST_RED){
                telemetry.addLine("Red Alliance");
            } else if (testAllianceStorage.getTestAlliance() == TestAllianceStorage.TestAlliance.TEST_BLUE){
                telemetry.addLine("Blue Alliance");
            } else {
                telemetry.addLine("No Alliance");
            }

            telemetry.addData("Pose: ", testAllianceStorage.getTestPose());
            telemetry.addData("SavedPose: ", testAllianceStorage.getTestPose());


        }
    }
}
