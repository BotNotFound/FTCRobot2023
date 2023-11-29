package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.OpBaseLinear;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.location.AprilTagLocator;
import org.firstinspires.ftc.teamcode.modules.location.LocalizedMovement;

@Autonomous(name = "Autonomous Main (Drive To Position)")
public class AutonomousMainReal extends OpBaseLinear {
    private AprilTagLocator aprilTagLocator;

    public static final double DISTANCE_TO_TEAM_PROP_U = 50.0;

    @Override
    public void initHardware() throws InterruptedException {
        super.initHardware();
        aprilTagLocator = new AprilTagLocator(this);
    }

    @Override
    public void runOpMode() {
        // idle
        arm.extendTo(LinearSlide.Presets.IDLE);
        arm.rotateJoint(Arm.Presets.IDLE);

        driveTrain.driveToOdometry(Movement.Axis.Y.genPointFromAxis(DISTANCE_TO_TEAM_PROP_U)); // drive to the U

        // determine what side team prop is on
        final double leftConfidence, middleConfidence, rightConfidence;
        middleConfidence = 3; // TODO check w/ vision module
        driveTrain.driveToOdometry(Movement.Axis.THETA.genPointFromAxis(-Math.PI / 2));
        leftConfidence = 0; // TODO see above
        driveTrain.driveToOdometry(Movement.Axis.THETA.genPointFromAxis(Math.PI));
        rightConfidence = 0; // TODO see above

        if (middleConfidence > leftConfidence && middleConfidence > rightConfidence) { // middle is most confident
            aprilTagLocator.setTagId(5);
        }
        else if (leftConfidence > rightConfidence) { // left is most confident, or equal with middle
            aprilTagLocator.setTagId(4);
        }
        else { // right is most confident, or all are equal
            aprilTagLocator.setTagId(6);
        }

        // use location of team prop to determine path
        driveTrain.driveToOdometry(driveTrain.getLocation().add(Movement.Axis.THETA.genPointFromAxis(-Math.PI / 2)));

        driveTrain.driveToOdometry(new Movement(0, 0, Math.PI / 2)); // rotate so we can see the AprilTags
        // AprilTagLocator only works if we can see the April Tag, So we convert its location to a more reliable locator
        LocalizedMovement aprilTagLoc_odom = new LocalizedMovement(0, 1, aprilTagLocator).convertToOtherLocator(driveTrain);
        driveTrain.driveTo(aprilTagLoc_odom);

        arm.extendTo(LinearSlide.Presets.ONE_SLIDE_USED);
        arm.rotateJoint(Arm.Presets.READY_FOR_SCORE);
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
