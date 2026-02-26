package org.firstinspires.ftc.teamcode.RR;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.List;

/** Field overlay drawing used by MainDrive Dashboard. */
public final class Drawing {
    private Drawing() {}

    /** Robot as circle + heading line (default blue). */
    public static void drawRobot(Canvas c, Pose2d t) {
        drawRobot(c, t, 1, "#3F51B5");
    }

    /** Draw robot with given stroke width and color (e.g. "#3F51B5" blue, "#FF5722" orange for camera pose). */
    public static void drawRobot(Canvas c, Pose2d t, double strokeWidth, String strokeColor) {
        final double ROBOT_RADIUS = 9;
        c.setStrokeWidth((int) strokeWidth);
        c.setStroke(strokeColor);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);
        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    /** Draw goal point (small filled circle). */
    public static void drawGoal(Canvas c, double goalX, double goalY, String color) {
        c.setStrokeWidth(1);
        c.setStroke(color);
        c.setFill(color);
        c.strokeCircle(goalX, goalY, 4);
        c.fillCircle(goalX, goalY, 3);
    }

    /** Draw line from robot to goal (turret aim line). */
    public static void drawRobotToGoalLine(Canvas c, Pose2d robot, double goalX, double goalY, String color) {
        c.setStrokeWidth(2);
        c.setStroke(color);
        c.strokeLine(robot.position.x, robot.position.y, goalX, goalY);
    }

    /** Draw camera relocalization pose on field (inches): distinct filled circle so it’s visible. */
    public static void drawCameraPose(Canvas c, Pose2d cameraPose, String color) {
        if (cameraPose == null) return;
        final double RADIUS = 12;
        c.setStrokeWidth(2);
        c.setStroke(color);
        c.setFill(color);
        c.fillCircle(cameraPose.position.x, cameraPose.position.y, RADIUS);
        c.strokeCircle(cameraPose.position.x, cameraPose.position.y, RADIUS);
    }

    /** Draw pose history as a polyline (localization path). */
    public static void drawPoseHistory(Canvas c, List<Pose2d> history, String color) {
        if (history == null || history.size() < 2) return;
        double[] x = new double[history.size()];
        double[] y = new double[history.size()];
        for (int i = 0; i < history.size(); i++) {
            x[i] = history.get(i).position.x;
            y[i] = history.get(i).position.y;
        }
        c.setStrokeWidth(1);
        c.setStroke(color);
        c.strokePolyline(x, y);
    }
}
