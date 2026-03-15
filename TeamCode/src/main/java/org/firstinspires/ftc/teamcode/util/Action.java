package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * Single step that runs every tick until it returns false. Used for autonomous sequences (replaces Road Runner Action).
 */
public interface Action {
    /** @return true to keep running, false when done */
    boolean run(TelemetryPacket packet);
}
