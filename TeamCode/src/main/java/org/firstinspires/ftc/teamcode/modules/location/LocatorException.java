package org.firstinspires.ftc.teamcode.modules.location;

public class LocatorException extends RuntimeException {
    public static final String ERROR_MESSAGE = "Could not use locator to get location";

    private final Locator locator;

    public Locator getLocator() {
        return locator;
    }

    public LocatorException(Locator locator) {
        super(ERROR_MESSAGE);
        this.locator = locator;
    }

    public LocatorException(Locator locator, Throwable cause) {
        super(ERROR_MESSAGE + ": An internal exception occurred", cause);
        this.locator = locator;
    }

    public LocatorException(Locator locator, String message) {
        super(ERROR_MESSAGE + ": " + message);
        this.locator = locator;
    }

    public LocatorException(Locator locator, String message, Throwable cause) {
        super(ERROR_MESSAGE + ": " + message, cause);
        this.locator = locator;
    }
}
