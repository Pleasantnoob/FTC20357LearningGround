package teamcode.OpModes.Auto.Solo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.ComplexMath.Storage.PoseStorage;

/** Garden + flowers, red side. Starts by the red garden. Everything else is in GardenFlowersAuto. */
@Autonomous(name = "Red Garden + Flowers", group = "Red")
public class RedGardenFlowersAuto extends GardenFlowersAuto {
    @Override
    protected PoseStorage.Alliance alliance() { return PoseStorage.Alliance.RED; }
}
