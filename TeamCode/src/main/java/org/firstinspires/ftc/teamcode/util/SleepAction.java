package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Action that runs for a fixed duration. */
public class SleepAction implements Action {
    private final double seconds;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean started;

    public SleepAction(double seconds) {
        this.seconds = seconds;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!started) {
            timer.reset();
            started = true;
        }
        return timer.seconds() < seconds;
    }
}
