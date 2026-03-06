package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

/**
 * Fades LED between min and max values during autonomous. Run in parallel with main sequence.
 * Stops when keepRunning returns false (e.g. when SetFlagAndEndAction runs).
 */
public class LedFadeAction implements Action {
    private static final double LED_MIN = 0.2799;
    private static final double LED_MAX = 0.728;
    private static final double FADE_PERIOD_SEC = 2.0;

    private final Servo led;
    private final BooleanSupplier keepRunning;
    private final ElapsedTime timer = new ElapsedTime();

    public LedFadeAction(Servo led, BooleanSupplier keepRunning) {
        this.led = led;
        this.keepRunning = keepRunning;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket p) {
        if (!keepRunning.getAsBoolean()) return false;
        double t = timer.seconds() / FADE_PERIOD_SEC;
        double pos = LED_MIN + (LED_MAX - LED_MIN) * (0.5 + 0.5 * Math.sin(2 * Math.PI * t));
        led.setPosition(pos);
        return true;
    }
}
