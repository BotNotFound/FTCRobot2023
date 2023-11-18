package org.firstinspires.ftc.teamcode.modules.location;

/**
 * Used to determine the origin of a {@link Locator}'s coordinate plane
 */
public enum LocatorKind {
    /**
     * The origin is the bottom-left corner in the field of play
     */
    FIELD_RELATIVE,
    /**
     * The origin is the starting position of the robot
     */
    ROBOT_START_RELATIVE,
    /**
     * The locator has no absolute coordinate plane of its own, and therefore has no origin
     */
    NO_ABSOLUTE_POSITION,
    /**
     * The origin is a specific object on the field of play
     */
    OBJECT_RELATIVE
}
