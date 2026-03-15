package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.Arrays;
import java.util.List;

/** Runs actions one after another. */
public class SequentialAction implements Action {
    private final List<Action> actions;
    private int index = 0;

    public SequentialAction(Action... actions) {
        this.actions = Arrays.asList(actions);
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        while (index < actions.size()) {
            if (!actions.get(index).run(packet)) {
                index++;
                continue;
            }
            return true;
        }
        return false;
    }
}
