package org.firstinspires.ftc.teamcode.modules.location;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.modules.ModuleBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public abstract class TeamPropLocator extends ModuleBase implements Locator {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    /**
     * Name of the model used, stored in FtcRobotController/src/main/assets
     */
    private static final String TFOD_MODEL_ASSET = "model_unquantNP.tflite";

    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Pixel",
    };

    /**
     * List of currently detected objects. Ideally, in games, there is only one object in this list.
     */
    List<Recognition> currentRecognitions;

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public TeamPropLocator(@NonNull OpMode registrar) throws InterruptedException {
        super(registrar);
        Telemetry telemetry = registrar.telemetry;
        HardwareMap hardwareMap = registrar.hardwareMap;

        initTfod(hardwareMap);

//        while(visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){
//
//        }

        sleep(5000);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();



        telemetryTfod(telemetry);

        // Push telemetry to the Driver Station.
        telemetry.update();



        // Save CPU resources; can resume streaming when needed.
        /*if (gamepad1.dpad_down) {
            visionPortal.stopStreaming();
        } else if (gamepad1.dpad_up) {
            visionPortal.resumeStreaming();
        }*/

        // Share the CPU.
        sleep(20);

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod(HardwareMap hardwareMap) {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.FRONT);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.5f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod(Telemetry telemetry) {

        currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

    /**
     * Stops streaming camera view to (I don't know)
     */
    public void stopStreaming() {
        visionPortal.stopStreaming();
    }

    /**
     * Resumes streaming camera view to (I don't know)
     */
    public void resumeStreaming() {
        visionPortal.resumeStreaming();
    }

    public Recognition getStrongestRecognition() throws LocatorException{
        float highestConfidence = 0f;
        Recognition currentStrongest = null;
        for(Recognition recognition : currentRecognitions) {
            float confidence = recognition.getConfidence();
            if(confidence > highestConfidence) {
                currentStrongest = recognition;
            }
        }

        if(currentStrongest != null) {
            return currentStrongest;
        } else {
            throw new LocatorException(this, "No objects have been detected.");
        }
    }

    @Override
    public void cleanupModule() {
        visionPortal.close();
    }

    @Override
    public LocalizedMovement getLocation() throws LocatorException {
        Recognition recognition = getStrongestRecognition();
        double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
        double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
        // Focal length is in millimeters (currently 30 millimeters
        // how far in front the object is from the camera
        double distance_z = (76 * 30) / recognition.getWidth();
        // how far to the side the object is from the camera
        double distance_x = (distance_z / 30) *
                (x - 640);
        // how far above the object is from the camera
        double distance_y = (distance_z / 30) *
                (y - 360);

        return new LocalizedMovement(distance_x, distance_z, 0, this);
    }

    /*public PROP_LOCATION getSimpleLocation() {
        Recognition recognition = getStrongestRecognition();

        double x = (recognition.getLeft() + recognition.getRight()) / 2 ;

        if(x < 426) {
            return PROP_LOCATION.LEFT;
        } else if (x < 853) {
            return PROP_LOCATION.CENTER;
        } else if (x <= 1280) {
            return  PROP_LOCATION.RIGHT;
        }

        return PROP_LOCATION.UNKNOWN;
    }*/

    @Override
    public boolean isActive() {
        return false;
    }

    @Override
    public LocatorKind getKind() {
        return LocatorKind.OBJECT_RELATIVE;
    }

    @Override
    public LocalizedMovement convertFromOtherLocator(Movement distance, Locator other) {
        return Locator.super.convertFromOtherLocator(distance, other);
    }

    @Override
    public Movement getFieldSize() {
        return new Movement(
                3657.6,
                3657.6
        );
    }
}

enum PROP_LOCATION {
    LEFT, CENTER, RIGHT, UNKNOWN
}
