package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpBaseLinear;
import org.firstinspires.ftc.teamcode.modules.computervision.testing.ColorMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous(name = "Autonomous Control")
public class AutonomousMain extends OpBaseLinear {

    private ColorMassDetectionProcessor cmdProcessor = new ColorMassDetectionProcessor(new Scalar(0, 0, 150), new Scalar(180, 72, 255), () -> 50, () -> 426, () -> 853);
    private VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain.setVelocity(0,1,0);
        Thread.sleep(3000, 0);
        driveTrain.setVelocity(0,0,0);
    }
    
}
