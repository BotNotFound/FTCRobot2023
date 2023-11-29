package org.firstinspires.ftc.teamcode.modules.location;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class BluePropLocator extends TeamPropLocator {

    /**
     * Name of the model used, stored in FtcRobotController/src/main/assets
     */
    private static final String TFOD_MODEL_ASSET = "RedProp.tflite";

    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "RedProp",
    };

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public BluePropLocator(@NonNull OpMode registrar) throws InterruptedException {
        super(registrar);
    }
}
