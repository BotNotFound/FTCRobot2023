package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

/**
 * A point.  The coordinate system we are using is horizontal and vertical translation in a rotated
 *  XY coordinate plane, and our unit of distance is nanoseconds.
 */
public final class Point {

    public enum Axis {
        X,
        Y,
        ROTATION;

        public Point genPointFromAxis(double distance) {
            switch (this) {
                case X:
                    return new Point(distance, 0, 0);
                case Y:
                    return new Point(0, distance, 0);
                case ROTATION:
                    return new Point(0, 0, distance);

                default:
                    throw new RuntimeException("Invalid axis!");
            }
        }
    }

    public double x;
    public double y;
    public double rotation;

    public Point(double x, double y, double rotation) {
        this.x = x;
        this.y = y;
        this.rotation = rotation;
    }

    public Point() {
        this(0,0,0);
    }

    /**
     * Adds two points together (doesn't modify existing points)
     * @param addend the other addend
     * @return the sum of the two points
     */
    public Point add(Point addend) {
        return new Point(x + addend.x, y + addend.y, rotation + addend.rotation);
    }

    /**
     * Negates all coordinates in the point (doesn't modify the existing point)
     * @return the negated Point
     */
    public Point negate() {
        return multiply(-1);
    }

    /**
     * Multiplies two points together (doesn't modify existing points)
     * @param factor the other factor
     * @return the product of the two points
     */
    public Point multiply(double factor) {
        return new Point(x * factor, y * factor, rotation * factor);
    }

    @NonNull
    @Override
    public String toString() {
        return "<" + x + ", " + y + ", " + rotation + ">";
    }
}
