package org.firstinspires.ftc.teamcode.modules.location;

import org.firstinspires.ftc.teamcode.Movement;

public interface Locator {
    /**
     * Gets the current position of the robot
     * @return The robot's current position, in whatever units are used by this locator
     * @throws LocatorException The locator could not get the current position
     *  due to an internal error
     * @throws NoAbsolutePositionException The locator has no absolute position
     * @see #isActive()
     * @see #getKind()
     */
    LocalizedMovement getLocation() throws LocatorException;

    /**
     * {@link #getLocation()} will always throw an exception if this is false
     * @return True if the locator currently functional, otherwise false
     */
    boolean isActive();

    /**
     * Gets the type of the locator.  This is used to determine
     *  what the provided coordinates are relative to
     * @return The locator's type
     */
    LocatorKind getKind();

    /**
     * Converts a {@link Movement} object from another coordinate system to the one used by
     * this locator
     * @param distance the {@link Movement} object to convert
     * @param other the {@link Locator} object that contains the coordinate system currently used
     *              by {@code distance}
     * @return the point in this locator's coordinate system
     */
    default LocalizedMovement convertFromOtherLocator(Movement distance, Locator other) {
        if (distance == null) {
            return LocalizedMovement.construct(Movement.zero(), this);
        }
        Movement otherFieldSize = other.getFieldSize();
        Movement thisFieldSize = getFieldSize();
        return new LocalizedMovement(
                distance.x / otherFieldSize.x * thisFieldSize.x,
                distance.y / otherFieldSize.y * thisFieldSize.y,
                distance.rotation,
                this
        );
    }

    /**
     * Converts a {@link LocalizedMovement} object from another coordinate system to the one used by
     * this locator
     * @param localizedDistance the {@link LocalizedMovement} to convert
     * @return the point in this locator's coordinate system
     * @see LocalizedMovement
     */
    default LocalizedMovement convertFromOtherLocator(LocalizedMovement localizedDistance) {
        if (localizedDistance.getLocator().equals(this)) {
            return localizedDistance;
        }
        else {
            return convertFromOtherLocator(localizedDistance, localizedDistance.getLocator());
        }
    }

    /**
     * Gets the size of the field, in whatever units are used by this locator
     * @return The size of the field
     */
    Movement getFieldSize();
}
