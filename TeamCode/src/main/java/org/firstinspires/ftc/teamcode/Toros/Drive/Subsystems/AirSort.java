package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Airsort: motif from AprilTags 21/22/23; shot decision from launcher color only (c3) for consistency.
 * We use only the top/launcher sensor (c3): "is the current ball at the launcher correct for this shot?"
 * Index resets when airsort is toggled on. c1/c2 are not used for the decision (optional for telemetry).
 */
public class AirSort {

    public enum ShotMode { FAST_SHOT, SLOW_SHOT }

    private static final String GREEN = "green";
    private static final String PURPLE = "purple";
    private static final String UNKNOWN = "unknown";

    /** Minimum RGB sum to consider a ball present (avoid empty as "purple"). */
    private static final int BALL_PRESENT_THRESHOLD = 80;
    /** Green: dominant G over R and B. */
    private static final int GREEN_DOMINANCE_THRESHOLD = 30;

    private final AprilTagProcessor aprilTag;
    private final ColorSensor c1, c2, c3;

    /** Desired order for the three shots (from obelisk tags 21, 22, 23). Index 0 = first shot. Stored once seen; not overwritten. */
    private final String[] motif = new String[] { PURPLE, PURPLE, PURPLE };
    /** Once we've seen tag 21/22/23 and set motif, stop reading AprilTag for motif (use stored value for rest of run). */
    private boolean motifStored = false;
    /** Ball at launcher only (c3). We ignore c1/c2 for the shot decision for consistency. */
    private String ballAtLauncher = UNKNOWN;

    /** Which shot in the sequence we're on (0, 1, or 2). */
    private int shotIndex = 0;

    /** True for one cycle after we detect ball left launcher (used for auto-advance). */
    private boolean shotDetectedThisUpdate = false;
    /** Previous "ball at launcher" state to detect exit. */
    private String lastBallAtLauncher = UNKNOWN;
    /** When true, advance shot index automatically (time-based after shot detected). */
    private boolean autoAdvanceOnBallExit = true;
    /** Time-based advance: when we detect shot, schedule index advance after this many ms. */
    private static final long DEFAULT_ADVANCE_DELAY_MS = 250;
    private long advanceDelayMs = DEFAULT_ADVANCE_DELAY_MS;
    /** If >= 0, advance index when currentTimeMillis() >= this time. */
    private long advanceIndexAtMillis = -1;

    public AirSort(AprilTagProcessor aprilTag, ColorSensor c1, ColorSensor c2, ColorSensor c3) {
        this.aprilTag = aprilTag;
        this.c1 = c1;
        this.c2 = c2;
        this.c3 = c3;
    }

    /**
     * Call every loop. Updates motif, launcher ball (c3 only), and optionally detects shot and advances index (time-based).
     */
    public void update() {
        shotDetectedThisUpdate = false;
        updateMotifFromAprilTag();
        ballAtLauncher = classifyColor(c3);

        // Time-based advance: when scheduled time is reached, advance index (faster than waiting for next ball at launcher).
        if (autoAdvanceOnBallExit && advanceIndexAtMillis >= 0 && System.currentTimeMillis() >= advanceIndexAtMillis) {
            advanceToNextShot();
            advanceIndexAtMillis = -1;
        }

        String atLauncher = getBallAtLauncher();
        boolean hadBall = !UNKNOWN.equals(lastBallAtLauncher);
        boolean hasBall = !UNKNOWN.equals(atLauncher);
        if (hadBall && !hasBall) {
            shotDetectedThisUpdate = true;
            if (autoAdvanceOnBallExit) {
                advanceIndexAtMillis = System.currentTimeMillis() + advanceDelayMs;
            }
        }
        lastBallAtLauncher = ballAtLauncher;
    }

    private void updateMotifFromAprilTag() {
        if (motifStored) return;  // use stored motif; don't look at tag again
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            switch (d.id) {
                case 21:
                    motif[0] = GREEN;
                    motif[1] = PURPLE;
                    motif[2] = PURPLE;
                    motifStored = true;
                    return;
                case 22:
                    motif[0] = PURPLE;
                    motif[1] = GREEN;
                    motif[2] = PURPLE;
                    motifStored = true;
                    return;
                case 23:
                    motif[0] = PURPLE;
                    motif[1] = PURPLE;
                    motif[2] = GREEN;
                    motifStored = true;
                    return;
                default:
                    break;
            }
        }
    }

    /** True if motif was already read from AprilTag 21/22/23 this run (stored, no need to look again). */
    public boolean isMotifStored() {
        return motifStored;
    }

    /** Clear stored motif so next tag 21/22/23 seen will be read again (e.g. new match). */
    public void clearStoredMotif() {
        motifStored = false;
        motif[0] = PURPLE;
        motif[1] = PURPLE;
        motif[2] = PURPLE;
    }


    private static String classifyColor(ColorSensor s) {
        int r = s.red();
        int g = s.green();
        int b = s.blue();
        if (r + g + b < BALL_PRESENT_THRESHOLD) return UNKNOWN;
        if (g > r + GREEN_DOMINANCE_THRESHOLD && g > b + GREEN_DOMINANCE_THRESHOLD) return GREEN;
        return PURPLE;
    }

    /** Returns the desired color order (green/purple) for the three shots. Index 0 = first shot. */
    public String[] getMotif() {
        return motif;
    }

    /** Ball at launcher (sensor c3 only — single source of truth for fast/slow shot). */
    public String getBallAtLauncher() {
        return ballAtLauncher;
    }

    /** Set which shot in the sequence we're on (0, 1, or 2). Clears any pending time-based advance. */
    public void setShotIndex(int index) {
        shotIndex = Math.max(0, Math.min(2, index));
        advanceIndexAtMillis = -1;
    }

    /**
     * Call when you know a shot was fired (e.g. from shoot button / launcher trigger).
     * Advances shot index: 0→1, 1→2, 2 stays 2. Returns the new shot index.
     */
    public int advanceToNextShot() {
        if (shotIndex < 2) shotIndex++;
        return shotIndex;
    }

    /** True for one update cycle after we detected ball left launcher (ball-at-launcher went from present to absent). */
    public boolean wasShotDetected() {
        return shotDetectedThisUpdate;
    }

    /** When true (default), shot index advances automatically after shot is detected, using time-based delay. */
    public void setAutoAdvanceOnBallExit(boolean enable) {
        autoAdvanceOnBallExit = enable;
    }

    public boolean isAutoAdvanceOnBallExit() {
        return autoAdvanceOnBallExit;
    }

    /** Delay in ms from "shot detected" (ball left launcher) until index advances. Default 250 ms. Tune if index was too slow. */
    public void setAdvanceDelayMs(long delayMs) {
        this.advanceDelayMs = Math.max(0, delayMs);
    }

    public long getAdvanceDelayMs() {
        return advanceDelayMs;
    }

    /** Current shot index (0 = first shot, 1 = second, 2 = third). */
    public int getShotIndex() {
        return shotIndex;
    }

    /**
     * Correct color for the current shot (based on motif and shotIndex).
     */
    public String getDesiredColorForCurrentShot() {
        return motif[shotIndex];
    }

    /**
     * True if the ball at the launcher matches the desired color for the current shot → use fast shot.
     * False → use slow shot.
     */
    public boolean useFastShot() {
        String atLauncher = getBallAtLauncher();
        String desired = getDesiredColorForCurrentShot();
        if (UNKNOWN.equals(atLauncher)) return false;
        return desired.equals(atLauncher);
    }

    /**
     * Returns FAST_SHOT if ball at launcher is correct color for current shot, else SLOW_SHOT.
     */
    public ShotMode getShotMode() {
        return useFastShot() ? ShotMode.FAST_SHOT : ShotMode.SLOW_SHOT;
    }
}
