package org.firstinspires.ftc.teamcode.modules.location;

import org.firstinspires.ftc.teamcode.Movement;

/**
 * A variant of {@link Movement} that specifies its coordinate system via a provided {@link Locator}
 */
public final class LocalizedMovement extends Movement {
    private final Locator locator;

    public Locator getLocator() {
        return locator;
    }

    public LocalizedMovement(double x, double y, double rotation, Locator locator) {
        super(x, y, rotation);
        this.locator = locator;
    }

    public LocalizedMovement(double x, double y, Locator locator) {
        this(x, y, 0, locator);
    }

    public static LocalizedMovement construct(Movement movement, Locator locator) {
        return new LocalizedMovement(movement.x, movement.y, movement.rotation, locator);
    }

    public LocalizedMovement convertToOtherLocator(Locator other) {
        return other.convertFromOtherLocator(this);
    }
}
