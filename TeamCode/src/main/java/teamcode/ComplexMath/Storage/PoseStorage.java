package teamcode.ComplexMath.Storage;

/**
 * Carries the robot's position and alliance from auto into TeleOp.
 * Everything is static, so every "new PoseStorage()" shares the same values, and they
 * survive between OpModes as long as the robot controller app isn't restarted.
 * Auto saves here every loop; TeleOp reads it at init to know where the robot is.
 */
public class PoseStorage {

    // volatile saves across threads, static keeps it constant across all classes
    private static volatile double PosX = 0;
    private static volatile double PosY = 0;
    private static volatile double robotHeading = 0;
    //use this so the turretHeading also saves across autos to teleOps, or I could just set the position as 0 at the end of auto.
    private static volatile double turretHeading = 0;

    public enum Alliance {RED, BLUE, NONE}
    private static volatile Alliance alliance = Alliance.NONE;




    public void savePoseToStorage(double updatedX, double updatedY, double updatedHeading){
        PosX = updatedX;
        PosY = updatedY;
        robotHeading = updatedHeading;
    }

    public double getPosX(){

        return PosX;
    }

    public double getPosY(){
        return PosY;

    }

    public double getRobotHeading(){
        return robotHeading;

    }


    public void setAlliance(Alliance newAlliance){

        alliance = newAlliance;
    }
    public Alliance getAlliance(){
        return alliance;
    }









}
