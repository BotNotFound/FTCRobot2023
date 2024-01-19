package org.firstinspires.ftc.teamcode.opmode.autonomous.blue.wall;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.location.AprilTagLocator;
import org.firstinspires.ftc.teamcode.opmode.autonomous.blue.AutonomousBlueImpl;
import org.firstinspires.ftc.teamcode.opmode.autonomous.template.AutonomousConstants;

public abstract class AutonomousBlueWallStartImpl extends AutonomousBlueImpl {
    public static final String BLUE_WALL_AUTO_GROUP_NAME = BLUE_AUTO_GROUP_NAME + " | Wall";

    /**
     * Called when the robot is in the middle of the U of spike marks, facing the backdrop.  When implemented, moves the
     * robot to the backdrop so that it is directly in front of the target AprilTag.
     */
    @Override
    protected void driveToBackdrop() {
        final AprilTagLocator aprilTagLocator = getModuleManager().getModule(AprilTagLocator.class);
        final int tagId = aprilTagLocator.getTagId();
        TrajectoryBuilder builder = getDriverToPosition().trajectoryBuilder(getDriverToPosition().getPoseEstimate());
        if (tagId == getLeftAprilTagId()) {
            builder.strafeRight(AutonomousConstants.TILE_SIDE_LENGTH_IN);
        }
        else if (tagId == getRightAprilTagId()) {
            getDriverToPosition().turn(180);
        }
        else {
            getDriverToPosition().turn(-90);
        }

        builder.forward(AutonomousConstants.DISTANCE_FROM_FAR_SPIKE_MARKS_TO_BACKDROP_IN);

        final double thirdOfBackdropWidth = AutonomousConstants.BACKDROP_WIDTH / 3;

        if (tagId == getLeftAprilTagId()) {
            builder.strafeLeft(AutonomousConstants.TILE_SIDE_LENGTH_IN - (2 * thirdOfBackdropWidth));
        }
        else if (tagId == getRightAprilTagId()) {
            builder.strafeLeft(thirdOfBackdropWidth);
        }

        getDriverToPosition().followTrajectory(builder.build());
    }

    /**
     * Called after the target AprilTag is set, when the robot is in the U of spike marks, with the team prop directly
     * in front of it.  When implemented, moves the robot so that the pixel won't be placed on top of the team prop, and
     * places the first pixel on the spike mark.
     */
    @Override
    protected void scoreOnSpikeMark() throws InterruptedException {
        final TrajectoryBuilder builder = getDriverToPosition().trajectoryBuilder(getDriverToPosition().getPoseEstimate());

        builder.strafeRight(AutonomousConstants.SPIKE_MARK_WIDTH_IN / 3);

        getDriverToPosition().followTrajectory(builder.build());

        final Arm arm = getModuleManager().getModule(Arm.class);
        arm.rotateWristTo(Arm.WristPresets.IDLE);
        Thread.sleep(200);
        arm.rotateArmTo(Arm.ArmPresets.IDLE);
        Thread.sleep(200);
        arm.rotateArmTo(Arm.ArmPresets.DEPOSIT_ON_FLOOR);
        arm.rotateWristTo(Arm.WristPresets.DEPOSIT_ON_FLOOR);
        arm.cycleFlap();

        prepareArmForDriving();

        getDriverToPosition().followTrajectory(
                getDriverToPosition().trajectoryBuilder(getDriverToPosition().getPoseEstimate())
                        .strafeLeft(AutonomousConstants.SPIKE_MARK_WIDTH_IN / 3)
                        .build()
        );
    }
}
