package teamcode.Tests.AllianceStorageTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * (Hidden) Test for passing alliance/pose from auto to TeleOp. Pair with AllianceTestTeleOp.
 * NOTE: testAllianceStorage is never created, so this crashes as written.
 */
@Disabled
@Autonomous
public class AllianceTestAuto extends LinearOpMode {

    private TestAllianceStorage testAllianceStorage;

    @Override
    public void runOpMode() throws InterruptedException {

        double testX = 12;
        double testY = 13;
        double testHeading = 14;
        testAllianceStorage.setTestAlliance(TestAllianceStorage.TestAlliance.TEST_RED);

        waitForStart();



        while (opModeIsActive()) {



            testAllianceStorage.saveTestPose(testX, testY, testHeading);

        }
    }
}
