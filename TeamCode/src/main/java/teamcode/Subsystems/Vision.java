package teamcode.Subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


/**
 * AprilTag camera ("Webcam 1"). Nothing opens the camera unless initVisionPortal() is called,
 * so it costs nothing in OpModes that don't use it. Only ScanTest uses it right now.
 * BioBuzz tags (IDs 30-45) are on the BOTTOM of each cell facing down, so the camera
 * has to look up at the hive. Calibration is still the SDK default - see CAMERA CALIBRATION below.
 */
public class Vision {

    //UPDATE OUTLINE AFTER EVERY COMMIT

    // -------------------------------------------------------------------------------------
    // -------------------------------------------------------------------------------------
    // File Outline
    // -------------------------------------------------------------------------------------
    // - Global Definitions [Lines 30-60]
    // - Building The Processors (AprilTag, Colors) [Lines 65 - 160]
    // - Building VisionPortal [Lines 165 - 200]
    // - Enabling and Disabling Processors and VisionPortal Streaming [Lines 205 - 250]
    // -------------------------------------------------------------------------------------
    // -------------------------------------------------------------------------------------


    private final HardwareMap hardwareMap;
    public Vision(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    //may make this static so it can be changed; would have be changed to not final if changed to static
    public final boolean USE_WEBCAM = true;
    public final int RESOLUTION_WIDTH = 640;
    public final int RESOLUTION_HEIGHT = 480;

    /*
    //find a height greater than the lowered cell but lower than the higher cell
    double RAISED_CELL_HEIGHT_THRESHOLD = 0;
    //adjust this based on the direction of the camera on the robot, will probably be useless and should
    //double check how this works if the Webcam fails, but rememember to double check.

     */
    public final BuiltinCameraDirection CAMERA_DIRECTION = BuiltinCameraDirection.FRONT;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    List<AprilTagDetection> currentDetections;
    AprilTagClusterDetection clusterDet;



    private final double cameraXOffset = 0; // put X offset value
    private final double cameraYOffset = 0; // put Y Offset value
    private final double cameraZOffset = 0; // put Z Offset value
    private final double cameraYaw = 0; // put yaw of camera in degrees
    private final double cameraPitch = 0; // put pitch of camera in degrees
    private final double cameraRoll = 0; // put roll of camera in degrees

    //choose best distanceUnit
    private final Position cameraPosition = new Position(DistanceUnit.INCH,cameraXOffset, cameraYOffset, cameraZOffset, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, cameraYaw, cameraPitch, cameraRoll, 0);






    public void initAprilTag(){
        aprilTagProcessor = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                //.setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();
    }

    public void initVisionPortal(){
        // ---------------
        // BUILD PROCESSORS
        // ---------------
        //initColorProcessors();
        initAprilTag();
        // ------------------
        // BUILD VISION PORTAL
        // ------------------

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if(USE_WEBCAM){
            //put whatever configuration name we end up using for the camera here
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            //may not work because we dont have a camera phone, check if this is still valid
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        //check resolution in case something needs to be changed or if the previous resolution was wrong
        builder.setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT));
        builder.addProcessors(aprilTagProcessor);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        //builder.addProcessors(aprilTagProcessor, redNectarLocator, blueNectarLocator, pollenLocator);


        visionPortal = builder.build();

        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
        //visionPortal.setProcessorEnabled(redNectarLocator, false);
        //visionPortal.setProcessorEnabled(blueNectarLocator, false);
        //visionPortal.setProcessorEnabled(pollenLocator, false);






    }


    public void resetScanner(){
        clusterDet = null;
    }

    public void scanForClusters(){
        currentDetections = aprilTagProcessor.getDetections();
        for(AprilTagDetection detection : currentDetections){
            if(detection instanceof AprilTagClusterDetection){
                clusterDet = (AprilTagClusterDetection) detection;
            }
        }
    }



    public boolean checkIfStartingClusterRaised(){
        //if clusterDet != null, returns true; if clusterDet == null, returns false
        return clusterDet != null;


    }



















    public AprilTagClusterDetection getCurrentCluster(){
        return clusterDet;
    }




    //EVERYTHING AFTER SEEMS FINE, MAY NEED TO UPDATE PROCESSORS


    public void enableAprilTag(){
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
    }
    public void disableAprilTag(){
        visionPortal.setProcessorEnabled(aprilTagProcessor, false);
    }









    //REMINDER: DO NOT CLOSE VISION PORTAL IN THE MIDDLE OF OPMODE UNLESS PERMANENTLY
    //FINISHED WITH IT. IT WILL CAUSE CPU STRAIN TO CLOSE AND BUILD IT AGAIN.

    public void restVisionPortal(){
        visionPortal.stopStreaming();
    }

    public void reactivateVisionPortal(){
        visionPortal.resumeStreaming();
    }


    public void closeVisionPortal(){
        visionPortal.close();
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

}
