package teamcode.Utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import teamcode.Subsystems.Turret;

/**
 * Draws the robot, turret aim and targets on the FTC Dashboard field view.
 * Pedro coordinates (0-144, corner origin) are shifted to the dashboard's center origin.
 * If the drawing is rotated relative to the real field, change ROTATION_DEG (0/90/180/270)
 * or MIRROR_X in the dashboard.
 */
@Config
public class DashboardField {

    public static double ROTATION_DEG = 0;
    public static boolean MIRROR_X = false;
    public static double ROBOT_RADIUS_IN = 9;

    private DashboardField() {}

    /** One-liner for autos: draws the field plus a few graphs and sends them. */
    public static void send(double x, double y, double headingRad, Turret turret) {
        TelemetryPacket packet = new TelemetryPacket();
        draw(packet.fieldOverlay(), x, y, headingRad, turret);
        packet.put("x", x);
        packet.put("y", y);
        packet.put("heading deg", Math.toDegrees(headingRad));
        packet.put("turret cmd deg", turret.getTurretHeading());
        packet.put("turret actual deg", turret.getActualDeg());
        packet.put("distance to target", turret.getDistanceToTarget());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public static void draw(Canvas c, double x, double y, double headingRad, Turret turret) {
        // field border
        c.setStroke("#888888").setStrokeWidth(1);
        double[] a = p(0, 0), b = p(144, 0), d = p(144, 144), e = p(0, 144);
        c.strokePolygon(new double[]{a[0], b[0], d[0], e[0]}, new double[]{a[1], b[1], d[1], e[1]});

        // all four targets, the active one filled
        drawTarget(c, Turret.redHighX, Turret.redHighY, "#ff4444", turret);
        drawTarget(c, Turret.redLowX, Turret.redLowY, "#ff4444", turret);
        drawTarget(c, Turret.blueHighX, Turret.blueHighY, "#4488ff", turret);
        drawTarget(c, Turret.blueLowX, Turret.blueLowY, "#4488ff", turret);

        // virtual goal (shoot on the move): purple X where the turret is really aiming
        if (turret.isShootOnTheMove()) {
            double[] v = p(turret.getVirtualTargetX(), turret.getVirtualTargetY());
            c.setStroke("#AA00FF").setStrokeWidth(2);
            c.strokeLine(v[0] - 2.5, v[1] - 2.5, v[0] + 2.5, v[1] + 2.5);
            c.strokeLine(v[0] - 2.5, v[1] + 2.5, v[0] + 2.5, v[1] - 2.5);
        }

        // robot body + heading
        double[] r = p(x, y);
        c.setStroke("#3F51B5").setStrokeWidth(2);
        c.strokeCircle(r[0], r[1], ROBOT_RADIUS_IN);
        double[] hEnd = p(x + Math.cos(headingRad) * ROBOT_RADIUS_IN, y + Math.sin(headingRad) * ROBOT_RADIUS_IN);
        c.strokeLine(r[0], r[1], hEnd[0], hEnd[1]);

        // turret pivot
        double cos = Math.cos(headingRad), sin = Math.sin(headingRad);
        double px = x + cos * Turret.pivotForwardIn - sin * Turret.pivotLeftIn;
        double py = y + sin * Turret.pivotForwardIn + cos * Turret.pivotLeftIn;
        double[] pv = p(px, py);
        double len = Math.max(12, turret.getDistanceToTarget());

        // thin green = commanded aim, thick = where the turret actually is (green on target, orange not yet)
        double cmd = headingRad + Math.toRadians(turret.getTurretHeading());
        double act = headingRad + Math.toRadians(turret.getActualDeg());
        double[] cEnd = p(px + Math.cos(cmd) * len, py + Math.sin(cmd) * len);
        double[] aEnd = p(px + Math.cos(act) * len, py + Math.sin(act) * len);
        c.setStroke("#00C853").setStrokeWidth(1);
        c.strokeLine(pv[0], pv[1], cEnd[0], cEnd[1]);
        c.setStroke(turret.isOnTarget() ? "#00C853" : "#FF9100").setStrokeWidth(2);
        c.strokeLine(pv[0], pv[1], aEnd[0], aEnd[1]);
    }

    private static void drawTarget(Canvas c, double tx, double ty, String color, Turret turret) {
        double[] t = p(tx, ty);
        boolean active = Math.abs(tx - turret.getTurretTargetX()) < 1e-6
                && Math.abs(ty - turret.getTurretTargetY()) < 1e-6;
        c.setStroke(color).setFill(color).setStrokeWidth(1);
        if (active) c.fillCircle(t[0], t[1], 3);
        else c.strokeCircle(t[0], t[1], 3);
    }

    /** Pedro field inches -> dashboard canvas inches. */
    private static double[] p(double x, double y) {
        double dx = x - 72, dy = y - 72;
        if (MIRROR_X) dx = -dx;
        double rot = Math.toRadians(ROTATION_DEG);
        double cr = Math.cos(rot), sr = Math.sin(rot);
        return new double[]{dx * cr - dy * sr, dx * sr + dy * cr};
    }
}
