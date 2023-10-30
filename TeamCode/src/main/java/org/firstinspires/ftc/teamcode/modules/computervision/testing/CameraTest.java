package org.firstinspires.ftc.teamcode.modules.computervision.testing;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

@Autonomous(name="Vision Test")
public class CameraTest extends LinearOpMode {
    private RedPropThreshold redPropThreshold = new RedPropThreshold(); //Create an object of the VisionProcessor Class
    private VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(redPropThreshold)
                .build();


        waitForStart();
        //portal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d"));
        //portal.
        telemetry.addData("Prop Position", redPropThreshold.getPropPosition());
        telemetry.update();                        //Will output prop position on Driver Station Console


    }
}