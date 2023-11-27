package org.firstinspires.ftc.teamcode.modules.location;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Movement;

/**
 * A variant of {@link Movement} that specifies its coordinate system via a provided {@link Locator}
 */
public final class LocalizedMovement extends Movement {
    private final Locator locator;

    public Locator getLocator() {
        return locator;
    }

    public LocalizedMovement(double x, double y, double theta, Locator locator) {
        super(x, y, theta);
        this.locator = locator;
    }

    public LocalizedMovement(double x, double y, Locator locator) {
        this(x, y, 0, locator);
    }

    public static LocalizedMovement construct(Movement movement, Locator locator) {
        return new LocalizedMovement(movement.x, movement.y, movement.theta, locator);
    }

    public LocalizedMovement convertToOtherLocator(Locator other) {
        if (other.equals(locator)) {
            return this; // so we don't get stuck in unnecessary infinite recursion
        }
        return other.convertFromOtherLocator(this);
    }

    public LocalizedMovement add(LocalizedMovement other) {
        return construct(super.add(other.convertToOtherLocator(getLocator())), getLocator());
    }

    @Override
    public LocalizedMovement add(Movement other) {
        return construct(super.add(other), getLocator());
    }

    public LocalizedMovement subtract(LocalizedMovement subtrahend) {
        return construct(super.subtract(subtrahend.convertToOtherLocator(getLocator())), getLocator());
    }

    @Override
    public Movement subtract(Movement subtrahend) {
        return construct(super.subtract(subtrahend), getLocator());
    }

    public LocalizedMovement negate() {
        return multiply(-1);
    }

    public LocalizedMovement multiply(double factor) {
        return construct(super.multiply(factor), getLocator());
    }

    @NonNull
    @Override
    public String toString() {
        return "l" + super.toString();
    }
}
