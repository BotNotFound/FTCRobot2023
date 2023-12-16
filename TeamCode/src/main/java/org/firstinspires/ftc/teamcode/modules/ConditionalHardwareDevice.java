package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;
import java.util.function.Consumer;

/**
 * Represents a hardware device that may or may not be accessible.
 * Used to ensure functionality of the robot during testing, when not everything is always connected to the Control Hub.
 */
public final class ConditionalHardwareDevice<T extends HardwareDevice> {
    /**
     * The hardware device
     */
    private final T device;

    /**
     * Is the device accessible?
     */
    private final boolean available;

    /**
     * [internal] Constructs this class with the specified state
     * @param device The hardware device
     */
    private ConditionalHardwareDevice(T device) {
        this.device = device;
        available = device != null;
    }

    /**
     * Attempts to get a hardware device
     * @param hardwareMap The {@link HardwareMap} object to query
     * @param deviceClass The class of the hardware device
     * @param deviceName The name of the hardware device
     * @return A {@link ConditionalHardwareDevice} object with the retrieved hardware device, if it can be retrieved
     * @param <U> The type of the hardware device
     */
    public static <U extends HardwareDevice> ConditionalHardwareDevice<U> tryGetHardwareDevice(HardwareMap hardwareMap, Class<U> deviceClass, String deviceName) {
        try {
            return new ConditionalHardwareDevice<>(hardwareMap.get(deviceClass, deviceName));
        }
        catch (Throwable th) {
            if (th.getClass() != IllegalArgumentException.class) {
                throw th;
            }
            else {
                return new ConditionalHardwareDevice<>(null);
            }
        }
    }

    /**
     * Is the hardware device accessible?
     * @return True if the hardware device exists, otherwise false
     */
    public boolean isAvailable() {
        return available;
    }

    /**
     * Gets the hardware device
     * @return The hardware device
     * @throws NullPointerException The hardware device is inaccessible
     */
    public T requireDevice() {
        return Objects.requireNonNull(device);
    }

    /**
     * Runs provided code only if the hardware device is accessible
     * @param it The code to run
     */
    public void runIfAvailable(Consumer<T> it) {
        runIfAvailable(it, us -> {}); // do nothing if the device is unavailable
    }

    /**
     * Runs provided code only if the hardware device is accessible
     * @param runnable The code to run
     * @param onUnavailable A function to run if the device is unavailable
     */
    public void runIfAvailable(Consumer<T> runnable, Consumer<ConditionalHardwareDevice<T>> onUnavailable) {
        if (isAvailable()) {
            runnable.accept(device);
        }
        else {
            onUnavailable.accept(this);
        }
    }
}
