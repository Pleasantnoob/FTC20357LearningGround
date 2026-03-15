package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/** Action that runs once (run() returns false immediately after first call). */
public class InstantAction implements Action {
    private final Runnable runnable;
    private boolean ran;

    public InstantAction(Runnable runnable) {
        this.runnable = runnable;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!ran) {
            runnable.run();
            ran = true;
        }
        return false;
    }
}
