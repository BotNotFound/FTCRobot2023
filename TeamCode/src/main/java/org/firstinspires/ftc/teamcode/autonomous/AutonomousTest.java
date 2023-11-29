package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.OpBaseLinear;
import org.firstinspires.ftc.teamcode.modules.location.AprilTagLocator;
import org.firstinspires.ftc.teamcode.modules.location.LocalizedMovement;

@Autonomous(name = "Autonomous Test")
public final class AutonomousTest extends OpBaseLinear {
    private AprilTagLocator aprilTagLocator;

    @Override
    public void initHardware() throws InterruptedException {
        super.initHardware();
        aprilTagLocator = new AprilTagLocator(this);
    }

    @Override
    public void runOpMode() {
        arm.rotateJoint(30);
//        aprilTagLocator.setTagId(5);
//        driveTrain.driveToOdometry(new Movement(0, AutonomousMainReal.DISTANCE_TO_TEAM_PROP_U));
    }

    @Override
    public void stop() {
        super.stop();
        aprilTagLocator.cleanupModule();
    }
}
