package org.firstinspires.ftc.teamcode.modules.location;

import org.firstinspires.ftc.teamcode.Movement;

/**
 * The default locator -- used as more of a placeholder, its coordinate system goes from <0,0>
 *     to <1,1>
 */
public final class DefaultLocator implements Locator {
    @Override
    public Movement getLocation() throws LocatorException {
        throw new NoAbsolutePositionException(this);
    }

    @Override
    public boolean isActive() {
        return true;
    }

    @Override
    public LocatorKind getKind() {
        return LocatorKind.NO_ABSOLUTE_POSITION;
    }

    @Override
    public LocalizedMovement convertFromOtherLocator(Movement distance, Locator other) {
        if (distance == null) {
            return LocalizedMovement.construct(Movement.zero(), this);
        }

        Movement fieldSize = other.getFieldSize();
        return new LocalizedMovement(
                distance.x / fieldSize.x,
                distance.y / fieldSize.y,
                distance.rotation,
                this
        );
    }

    @Override
    public Movement getFieldSize() {
        return new Movement(1, 1);
    }
}
