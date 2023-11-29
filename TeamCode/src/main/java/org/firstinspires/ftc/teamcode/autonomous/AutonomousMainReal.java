package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.OpBaseLinear;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.location.AprilTagLocator;
import org.firstinspires.ftc.teamcode.modules.location.LocalizedMovement;

@Autonomous(name = "Autonomous Main (Drive To Position)")
public final class AutonomousMainReal extends OpBaseLinear {
    private AprilTagLocator aprilTagLocator;

    enum Side { LEFT, MIDDLE, RIGHT }

    @Override
    public void initHardware() throws InterruptedException {
        super.initHardware();
        aprilTagLocator = new AprilTagLocator(this);
    }

    @Override
    public void runOpMode() {
        Side side;

        arm.extendTo(LinearSlide.Presets.IDLE);
        arm.rotateJoint(Arm.Presets.IDLE);

        // use location of team prop to determine path
        side = Side.MIDDLE;
        aprilTagLocator.setTagId(5); // TODO when vision is done, do that

        driveTrain.driveToOdometry(new Movement(0, 5, 0)); // TODO Vision locator goes here

        driveTrain.driveToOdometry(new Movement(0, 0, Math.PI / 2)); // rotate so we can see the AprilTags
        // AprilTagLocator only works if we can see the April Tag, So we convert its location to a more reliable locator
        LocalizedMovement aprilTagLoc_odom = new LocalizedMovement(0, 1, aprilTagLocator).convertToOtherLocator(driveTrain);
        if (side == Side.RIGHT) {
            // TODO don't run into the team prop
        }
        driveTrain.driveTo(aprilTagLoc_odom);

        arm.extendTo(LinearSlide.Presets.ONE_SLIDE_USED);
        arm.rotateJoint(Arm.Presets.FACING_BACKDROP);
        doubleClaw.incrementClawState();

        park();
    }

    @Override
    public void stop() {
        super.stop();
        aprilTagLocator.cleanupModule();
    }
    
    public void park() {} // TODO finish
}
