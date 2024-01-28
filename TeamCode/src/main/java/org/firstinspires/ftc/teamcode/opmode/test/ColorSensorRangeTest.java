package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.modules.detection.Prop;
import org.firstinspires.ftc.teamcode.modules.detection.PropDetector;
import org.firstinspires.ftc.teamcode.opmode.OpBase;

@Autonomous(name="Color Sensor Range Test")
public class ColorSensorRangeTest extends OpBase {

    private PropDetector propDetector;

    private ColorRangeSensor sensor;

    public static final String SENSOR_NAME = "Color Sensor";

    private boolean propDetected;


    @Override
    protected void initModules() {
        propDetector = getModuleManager().getModule(PropDetector.class);
        sensor = ConditionalHardwareDevice.tryGetHardwareDevice(hardwareMap, ColorRangeSensor.class, SENSOR_NAME).requireDevice();
    }

    @Override
    public void loop() {
        if(propDetector.isPropDetected(Prop.RED_TEAM_PROP)) {
            propDetected = true;
        } else {
            propDetected = false;
        }
        telemetry.addData("Is Prop Detected: ", propDetected);
        telemetry.addData("Distance to object: ", sensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}
