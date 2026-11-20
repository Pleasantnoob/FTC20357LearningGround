package teamcode.Utility;

/**
 * Field size and frame, matching the Pedro visualizer (visualizer.pedropathing.com).
 *
 * The visualizer's BioBuzz field is 141.5 in between the walls (the real field is "about 144",
 * but tiles are a bit under 24 in), so the center is 70.75, not 72.
 * (0, 0) is the corner by the RED garden.
 *
 * The BioBuzz field is symmetric by a 180 deg rotation, not by a mirror. To get the other
 * alliance's version of a pose: x -> SIZE - x, y -> SIZE - y, heading + 180.
 */
public final class Field {
    private Field() {}

    /** Wall to wall, inches. */
    public static final double SIZE = 141.5;
    /** Field center (both x and y), inches. */
    public static final double CENTER = SIZE / 2;

    /** The other alliance's x for a point (180 deg rotation). */
    public static double rotX(double x) { return SIZE - x; }
    /** The other alliance's y for a point (180 deg rotation). */
    public static double rotY(double y) { return SIZE - y; }
}
