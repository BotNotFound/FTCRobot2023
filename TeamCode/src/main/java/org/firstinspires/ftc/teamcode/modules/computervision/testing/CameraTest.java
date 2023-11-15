package org.firstinspires.ftc.teamcode.modules.computervision.testing;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.function.DoubleSupplier;

@Autonomous(name="Vision Test")
public class CameraTest extends LinearOpMode {
    private RedPropThreshold redPropThreshold = new RedPropThreshold(); //Create an object of the VisionProcessor Class
    private ColorMassDetectionProcessor cmdProcessor = new ColorMassDetectionProcessor(new Scalar(0, 0, 150), new Scalar(180, 72, 255), () -> 50, () -> 213, () -> 426);
    private VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(cmdProcessor)
                .build();


        waitForStart();
        //portal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d"));
        //portal.
        telemetry.addData("Prop Position", cmdProcessor.getRecordedPropPosition());
        telemetry.update();                        //Will output prop position on Driver Station Console


    }
}