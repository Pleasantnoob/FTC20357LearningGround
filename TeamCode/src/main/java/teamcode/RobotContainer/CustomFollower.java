package teamcode.RobotContainer;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import teamcode.pedro.Constants;

/**
 * Not used right now. Old helper for making a Pedro follower at a start pose;
 * OpModes call Constants.create(hardwareMap) + setPose() directly instead.
 */
public class CustomFollower {

















    public Follower initCustomFollower(HardwareMap hardwareMap, Pose startPos){

        Follower follower = Constants.create(hardwareMap);
        follower.setPose(startPos);


        return follower;


    }






}
