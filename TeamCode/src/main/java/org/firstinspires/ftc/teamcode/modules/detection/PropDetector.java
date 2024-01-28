package org.firstinspires.ftc.teamcode.modules.detection;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.modules.core.Module;

public class PropDetector extends Module {
    /**
     * The sensor used to detect the prop
     */
    private final ConditionalHardwareDevice<ColorRangeSensor> sensor;

    /**
     * The name of the color sensor in the robot's configuration
     */
    public static final String SENSOR_NAME = "Color Sensor";

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public PropDetector(OpMode registrar) {
        super(registrar);
        sensor = ConditionalHardwareDevice.tryGetHardwareDevice(parent.hardwareMap, ColorRangeSensor.class, SENSOR_NAME);
        // status update
        sensor.runIfAvailable(
                device -> getTelemetry().addLine("[PropDetector] found ColorRangeSensor of type " + device.getDeviceName()),
                () -> getTelemetry().addLine("[PropDetector] could not find ColorRangeSensor!")
        );
    }

    /**
     * Returns whether the prop is there, based on the distance to the nearest object
     * @return Whether the prop is there or not as a boolean
     */
    public boolean isPropDetected() {
        return sensor.requireDevice().getDistance(DistanceUnit.CM) < 9.5;
    }

    /**
     * Returns whether the prop is there, based on the distance to the nearest object, as well as which RGB value (Red, Green, or Blue) is most present in the sensor's view(as a redundancy)
     * @param prop This is the prop type/color to look for
     * @return Whether the prop is there or not as a boolean
     */
    public boolean isPropDetected(Prop prop) {
        if(sensor.requireDevice().getDistance(DistanceUnit.CM) < 9.5) {
            return true;
        }
        switch (prop) {
            case RED_TEAM_PROP:
                return sensor.requireDevice().red() > sensor.requireDevice().green() && sensor.requireDevice().red() > sensor.requireDevice().blue();
            case BLUE_TEAM_PROP:
                return sensor.requireDevice().blue() > sensor.requireDevice().green() && sensor.requireDevice().blue() > sensor.requireDevice().red();
        }
        return false;
    }

    @Override
    public void cleanupModule() {

    }

    @Override
    public void log() {

    }
}
