package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

/**
 * A point.  The coordinate system we are using is horizontal and vertical translation in a rotated
 *  XY coordinate plane, and our unit of distance is nanoseconds.
 */
public final class Point {
    public static final double EPSILON = 0.001;

    /**
     * Determines if the point is small enough to be considered 'zero'
     * @return true if the point is small enough
     */
    public boolean isZero() {
        return Math.abs(x) < EPSILON &&
                Math.abs(y) < EPSILON &&
                Math.abs(rotation) < EPSILON;
    }

    @Override
    public boolean equals(@Nullable Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj.getClass() != Point.class) {
            return false;
        }
        return negate().add((Point)obj).isZero();
    }

    /**
     * The number of bits in an integer. Used by hashCode.
     * @see #hashCode()
     */
    private static final int LENGTH_OF_AN_INTEGER = (int)(Math.log(Integer.MAX_VALUE) / Math.log(2)) + 1; // MAX_VALUE is positive, so it will be 1 bit short
    /**
     * A bitmask for the first third of an integer. Used by hashCode.
     * @see #hashCode()
     */
    private static final int THIRD_INT_MASK = (Integer.MAX_VALUE << (2 * LENGTH_OF_AN_INTEGER / 3)) >> (2 * LENGTH_OF_AN_INTEGER / 3);

    @Override
    public int hashCode() {
        // only care about equality as precise as EPSILON
        long roundedX = Math.round(x / EPSILON);
        long roundedY = Math.round(y / EPSILON);
        long roundedRotation = Math.round(rotation / EPSILON);
        return (int)(
                ((roundedX & THIRD_INT_MASK) << ((2 * LENGTH_OF_AN_INTEGER) / 3)) |
                        (roundedY | THIRD_INT_MASK) << (LENGTH_OF_AN_INTEGER / 3) |
                        (roundedRotation | THIRD_INT_MASK)
        ); // unique enough for our nonexistent usage of this function
    }

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
