package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR.PinpointLocalizer;

public class Pinpoint{
    public static class Params {
        public double parYTicks =  -648.1950127810029   ; // y position of the parallel encoder (in tick units)
        public double perpXTicks = -3692.469269014993; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    public double inPerTick = 0.00194723378259707 ;

    /** Use after PinpointLocalizer has been constructed so the Pinpoint is configured once (avoids drift from multiple inits). */
    public Pinpoint(HardwareMap hardwareMap){
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // Do not configure or reset here — PinpointLocalizer is the single source of config and reset.
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
    public double getHeading(){
        return driver.getHeading(AngleUnit.RADIANS);
    }

    public double getVelx(){
        return driver.getVelX(DistanceUnit.INCH);
    }


    public double getVelY(){
        return driver.getVelY(DistanceUnit.INCH);
    }
}

