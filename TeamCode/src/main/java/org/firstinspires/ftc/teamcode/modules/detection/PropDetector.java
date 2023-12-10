package org.firstinspires.ftc.teamcode.modules.detection;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.modules.ModuleBase;
import org.jetbrains.annotations.NotNull;

public class PropDetector extends ModuleBase {
    private final ColorRangeSensor sensor;

    public static final String SENSOR_NAME = "Color Sensor";

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public PropDetector(@NonNull @NotNull OpMode registrar) {
        super(registrar);
        sensor = parent.hardwareMap.get(ColorRangeSensor.class, SENSOR_NAME);
    }

    public boolean isPropDetected(Prop prop) {
        return sensor.argb() == prop.argb;
    }

    @Override
    public void cleanupModule() {

    }

    @Override
    public void log() {

    }
}
