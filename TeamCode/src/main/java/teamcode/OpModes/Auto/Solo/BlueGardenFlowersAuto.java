package teamcode.OpModes.Auto.Solo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.ComplexMath.Storage.PoseStorage;

/**
 * Garden + flowers, blue side: the red auto rotated 180 deg (starts by the blue garden).
 * Everything else is in GardenFlowersAuto.
 */
@Autonomous(name = "Blue Garden + Flowers", group = "Blue")
public class BlueGardenFlowersAuto extends GardenFlowersAuto {
    @Override
    protected PoseStorage.Alliance alliance() { return PoseStorage.Alliance.BLUE; }
}
