package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.util.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

/**
 * Fades LED between min and max values during autonomous. Run in parallel with main sequence.
 * Stops when keepRunning returns false (e.g. when SetFlagAndEndAction runs).
 */
/** Implements both util.Action and RR Action so it works in Pedro and RR parallel actions. */
public class LedFadeAction implements Action, com.acmerobotics.roadrunner.Action {
    public static final double LED_MIN = 0.2799;
    public static final double LED_MAX = 0.728;
    public static final double FADE_PERIOD_SEC = 2.0;

    /** Sine fade position for elapsed time (seconds). Shared by auto actions and DemoTeleop. */
    public static double fadePosition(double elapsedSec) {
        double t = elapsedSec / FADE_PERIOD_SEC;
        return LED_MIN + (LED_MAX - LED_MIN) * (0.5 + 0.5 * Math.sin(2 * Math.PI * t));
    }

    /** Alias for teleop callers; same as {@link #fadePosition(double)}. */
    public static double positionAtSeconds(double elapsedSec) {
        return fadePosition(elapsedSec);
    }

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
        led.setPosition(fadePosition(timer.seconds()));
        return true;
    }
}
