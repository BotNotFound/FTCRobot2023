package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpBase;
import org.firstinspires.ftc.teamcode.modules.computervision.testing.ColorMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous(name = "Autonomous Control")
public class AutonomousMain extends OpBase {

    private ColorMassDetectionProcessor cmdProcessor = new ColorMassDetectionProcessor(new Scalar(0, 0, 150), new Scalar(180, 72, 255), () -> 50, () -> 426, () -> 853);
    private VisionPortal portal;

    @Override
    public void start() {
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(cmdProcessor)
                .build();
        //Just uncomment the code below and put the focal length of the camera in. The distances are the z, x, and y distances, in that order. Look at the method's documentation for a better explanation.
        //double[] distances = cmdProcessor.Distance_finder(Focal_Length);
        driveTrain.moveAndRotateRobot(0.003, -0.002, 3);
    }

    @Override
    public void loop() {

    }
    
}
