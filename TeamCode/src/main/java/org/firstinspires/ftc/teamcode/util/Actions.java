package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Runs actions in a loop (replaces Road Runner Actions.runBlocking).
 */
public final class Actions {
    private Actions() {}

    /** Run a single action until it returns false or op mode stops. */
    public static void runBlocking(LinearOpMode opMode, Action action) {
        TelemetryPacket packet = new TelemetryPacket();
        while (opMode.opModeIsActive() && action.run(packet)) {
            opMode.sleep(20);
        }
    }
}
