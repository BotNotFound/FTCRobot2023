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
    Movement getLocation() throws LocatorException;

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
    Movement convertFromOtherLocator(Movement distance, Locator other);

    /**
     * Gets the size of the field, in whatever units are used by this locator
     * @return The size of the field
     */
    Movement getFieldSize();
}
