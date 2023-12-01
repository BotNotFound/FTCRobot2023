package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpBaseLinear;
import org.firstinspires.ftc.teamcode.modules.location.*;

@Autonomous(name = "TensorFlow Testing")
public class AutonomousMainTFTesting extends OpBaseLinear {

    protected TeamPropLocator teamPropLocator;

    WebcamName webcamName;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    @Override
    public void initHardware() throws InterruptedException {
        super.initHardware();

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        teamPropLocator = new BluePropLocator(this);
    }
}
