/*package org.firstinspires.ftc.teamcode.modules.computervision;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.ModuleBase;
import org.firstinspires.ftc.teamcode.modules.computervision.testing.RedPropThreshold;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.openftc.easyopencv.*;

public class Camera extends ModuleBase{
    private OpenCvWebcam webcam;
    private HardwareMap hardwareMap;
    private RedPropThreshold p1; // sample pipeline
    private RedPropThreshold p2; // another sample pipeline

    public Camera(@NonNull OpMode registrar) throws InterruptedException { // hardware map from the base class is a parameter
        super(registrar);
        HardwareMap hw = registrar.hardwareMap;
        VisionProcessor
        p1 = new RedPropThreshold(); // initialize your pipeline classes
        p2 = new RedPropThreshold();

        this.hardwareMap = hw;    //Configure the Camera in hardwaremap
        int cameraMonitorViewId =
                hardwareMap
                        .appContext
                        .getResources()
                        .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Get camera from hardware map, replace 'camera' with what is in your controlhub
        webcam =
                OpenCvCameraFactory.getInstance()
                        .createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        webcam.setPipeline(p1); // Setting the intial pipeline

        webcam.setMillisecondsPermissionTimeout(2500);

        // Streaming Frames
        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {}
                });
    }

    // Switching Between Pipelines
    public void switchToSecondPipeline(){
        webcam.setPipeline(p2);
    }

    public void switchToFirstPipeline(){
        webcam.setPipeline(p1);
    }

    // Get information from pipeline
    public String getPipeline1Output(){
        return p1.getPropPosition();
    }

    // call stop at the end of the opMode.
    public void stop() {
        webcam.stopStreaming();
    }
}*/
