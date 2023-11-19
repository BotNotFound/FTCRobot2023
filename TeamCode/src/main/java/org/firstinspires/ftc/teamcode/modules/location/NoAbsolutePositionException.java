package org.firstinspires.ftc.teamcode.modules.location;

public final class NoAbsolutePositionException extends LocatorException {
    public NoAbsolutePositionException(Locator locator) {
        super(locator, "This locator is configured as NO_ABSOLUTE_POSITION");
    }
}
