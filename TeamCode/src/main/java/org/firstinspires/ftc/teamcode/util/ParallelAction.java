package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.Arrays;
import java.util.List;

/** Runs all actions every tick until every one has returned false. */
public class ParallelAction implements Action {
    private final List<Action> actions;

    public ParallelAction(Action... actions) {
        this.actions = Arrays.asList(actions);
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        boolean anyRunning = false;
        for (Action a : actions) {
            if (a.run(packet)) anyRunning = true;
        }
        return anyRunning;
    }
}
