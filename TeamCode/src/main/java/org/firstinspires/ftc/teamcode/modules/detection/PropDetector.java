package org.firstinspires.ftc.teamcode.modules.detection;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.teamcode.modules.ModuleBase;

public class PropDetector extends ModuleBase {
    private final ColorRangeSensor sensor;

    public static final String SENSOR_NAME = "Color Sensor";

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public PropDetector(OpMode registrar) {
        super(registrar);
        sensor = parent.hardwareMap.get(ColorRangeSensor.class, SENSOR_NAME);
    }

    public boolean isPropDetected(Prop prop) {
        /*float[] currentHSV = new float[3];
        Color.RGBToHSV(sensor.red(), sensor.green(), sensor.blue(), currentHSV);
        Color.RGBToHSV(prop.redChannel(), prop.greenChannel(), prop.blueChannel(), testHSV);
        if(currentHSV[1] >= testHSV[1] && currentHSV[0] >= testHSV[0] - colorPreciseness && currentHSV[0] <= testHSV[0] + colorPreciseness) {
            return true;
        }*/
        switch (prop) {
            case RED_TEAM_PROP:
                return sensor.red() > sensor.green() && sensor.red() > sensor.blue();
            case BLUE_TEAM_PROP:
                return sensor.blue() > sensor.green() && sensor.blue() > sensor.red();
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
