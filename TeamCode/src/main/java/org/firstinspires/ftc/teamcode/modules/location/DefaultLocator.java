package org.firstinspires.ftc.teamcode.modules.location;

import org.firstinspires.ftc.teamcode.Movement;

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
    public Movement convertFromOtherLocator(Movement distance, Locator other) {
        if (distance == null) {
            return Movement.zero();
        }
        return distance;
    }

    @Override
    public Movement getFieldSize() {
        return null;
    }
}
