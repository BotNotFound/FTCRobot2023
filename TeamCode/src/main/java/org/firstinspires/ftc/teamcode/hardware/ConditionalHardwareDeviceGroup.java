package org.firstinspires.ftc.teamcode.hardware;

import android.util.Pair;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Hashtable;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Represents a group of hardware devices that may or may not be available
 */
public final class ConditionalHardwareDeviceGroup {
    /**
     * The hardware devices.
     * Each device is saved with the name used by the hardware map to retrieve it (its key).
     * @see com.qualcomm.robotcore.hardware.HardwareMap#get(Class, String)
     */
    private final Hashtable<String, ConditionalHardwareDevice<?>> devices;

    /**
     * Is every device in this group available?
     */
    private final AtomicBoolean allAreAccessible;

    private final HardwareMap hardwareMap;

    /**
     * Constructs an empty device group
     * @param hardwareMap The {@link HardwareMap} object to use
     */
    public ConditionalHardwareDeviceGroup(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        devices = new Hashtable<>();
        allAreAccessible = new AtomicBoolean(true);
    }

    public boolean areAllDevicesAvailable() {
        return allAreAccessible.get();
    }

    /**
     * Attempts to add the specified hardware device to the group
     * @param deviceType A class representing the device
     * @param deviceName The name of the device
     * @param <T> The device's type
     */
    public <T extends HardwareDevice> void tryLoadDevice(Class<? extends T> deviceType, String deviceName) {
        if (devices.containsKey(deviceName) && devices.get(deviceName) != null) {
            return; // hardware device has already been successfully loaded
        }

        final ConditionalHardwareDevice<T> maybeDevice = ConditionalHardwareDevice.tryGetHardwareDevice(hardwareMap, deviceType, deviceName);
        final boolean deviceExists = maybeDevice.isAvailable();

        allAreAccessible.compareAndSet(true, deviceExists); // if this is already false, no need to set it to something else
        devices.put(deviceName, maybeDevice);
    }

    /**
     * Attempts to add the specified hardware devices to the group
     * @param deviceInfoSet A list of pairs.  The first element of each pair is the desired class of the hardware device,
     *                     and the second element is the device's name.  The devices will be loaded in the order they are
     *                      provided.
     */
    @SafeVarargs
    public final void tryLoadDevices(Pair<Class<? extends HardwareDevice>, String>... deviceInfoSet) {
        for (Pair<Class<? extends HardwareDevice>, String> deviceInfo : deviceInfoSet) {
            tryLoadDevice(deviceInfo.first, deviceInfo.second);
        }
    }

    /**
     * Retrieve a device loaded in this group
     * @param expectedClass The class of the hardware device's expected type
     * @param deviceName The name of the device
     * @return The device if it is available, loaded into this group, and has the expected type; otherwise null
     * @param <T> The expected type of the hardware device
     * @throws NullPointerException The hardware device is inaccessible
     * @throws ClassCastException The retrieved device is not of the expected type, and no cast can be made to convert it to said class
     */
    public <T extends HardwareDevice> T getLoadedDevice(Class<? extends T> expectedClass,String deviceName) {
        final ConditionalHardwareDevice<?> maybeDevice = devices.get(deviceName);
        return expectedClass.cast(maybeDevice.requireDevice());
    }

    /**
     * Executes the given code only if all devices in this group are available
     * @param runnable The code to run.
     * @param onUnavailable A function to run if the devices are unavailable
     */
    public void executeIfAllAreAvailable(Runnable runnable, Runnable onUnavailable) {
        if (areAllDevicesAvailable()) {
            runnable.run();
        }
        else {
            onUnavailable.run();
        }
    }

    /**
     * Executes the given code only if all devices in this group are available
     * @param runnable The code to run.
     */
    public void executeIfAllAreAvailable(Runnable runnable) {
        executeIfAllAreAvailable(runnable, () -> {});
    }
}
