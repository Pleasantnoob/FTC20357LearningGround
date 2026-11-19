package teamcode.Tests.AllianceStorageTest;

import com.pedropathing.math.Pose;

/**
 * Static storage used only by the AllianceTest OpModes. The real one is PoseStorage.
 */
public class TestAllianceStorage {



    public enum TestAlliance {TEST_RED, TEST_BLUE, TEST_NONE}
    private volatile TestAlliance testAlliance;

    private volatile double PosX;
    private volatile double PosY;
    private volatile double Heading;



    public void setTestAlliance(TestAlliance newTestAlliance){

        testAlliance = newTestAlliance;

    }

    public void saveTestPose(double updatedX, double updatedY, double updatedHeading){

        PosX = updatedX;
        PosY = updatedY;
        Heading = updatedHeading;


    }

    public Pose getTestPose(){

        return new Pose(PosX, PosY, Heading);
    }


    public TestAlliance getTestAlliance(){

        return testAlliance;

    }


}
