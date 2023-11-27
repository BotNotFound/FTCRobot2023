package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpBaseLinear;
import org.firstinspires.ftc.teamcode.modules.location.AprilTagLocator;
import org.firstinspires.ftc.teamcode.modules.location.LocalizedMovement;

@Autonomous(name = "Autonomous Main (Drive To Position)")
public final class AutonomousMainReal extends OpBaseLinear {
    private AprilTagLocator aprilTagLocator;

    @Override
    public void initHardware() throws InterruptedException {
        super.initHardware();
        aprilTagLocator = new AprilTagLocator(this);
    }

    @Override
    public void runOpMode() {
        aprilTagLocator.setTagId(5);
        driveTrain.driveTo(new LocalizedMovement(1,0,0, driveTrain));
    }

    @Override
    public void stop() {
        super.stop();
        aprilTagLocator.cleanupModule();
    }
}
