package org.firstinspires.ftc.teamcode.autonomous.red.backdrop;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.autonomous.red.AutonomousRedImpl;
import org.firstinspires.ftc.teamcode.autonomous.template.AutonomousConstants;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.location.AprilTagLocator;

public abstract class AutonomousRedBackdropStartImpl extends AutonomousRedImpl {
    public static final String RED_BACKDROP_AUTO_GROUP_NAME = RED_AUTO_GROUP_NAME + " | Backdrop";

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
            builder.strafeLeft(AutonomousConstants.TILE_SIDE_LENGTH_IN);
        }
        else if (tagId == getRightAprilTagId()) {
            getDriverToPosition().turn(180);
        }
        else {
            getDriverToPosition().turn(-90);
        }

        builder.forward(AutonomousConstants.DISTANCE_FROM_NEAR_SPIKE_MARKS_TO_BACKDROP_IN);

        final double thirdOfBackdropWidth = AutonomousConstants.BACKDROP_WIDTH / 3;

        if (tagId == getLeftAprilTagId()) {
            builder.strafeRight(AutonomousConstants.TILE_SIDE_LENGTH_IN - (2 * thirdOfBackdropWidth));
        }
        else if (tagId == getRightAprilTagId()) {
            builder.strafeRight(thirdOfBackdropWidth);
        }

        getDriverToPosition().followTrajectory(builder.build());
    }

    /**
     * Called after the target AprilTag is set, when the robot is in the U of spike marks, with the team prop directly
     * in front of it.  When implemented, moves the robot so that the pixel won't be placed on top of the team prop, and
     * places the first pixel on the spike mark.
     */
    @Override
    protected void scoreOnSpikeMark() {
        final TrajectoryBuilder builder = getDriverToPosition().trajectoryBuilder(getDriverToPosition().getPoseEstimate());

        builder.strafeRight(AutonomousConstants.SPIKE_MARK_WIDTH_IN / 3);

        getDriverToPosition().followTrajectory(builder.build());

        final Arm arm = getModuleManager().getModule(Arm.class);
        arm.rotateArmTo(Arm.ArmPresets.DEPOSIT_ON_FLOOR);
        arm.rotateWristTo(Arm.WristPresets.DEPOSIT_ON_FLOOR);
        arm.openFlap();

        prepareArmForDriving();

        getDriverToPosition().followTrajectory(
                getDriverToPosition().trajectoryBuilder(getDriverToPosition().getPoseEstimate())
                        .strafeLeft(AutonomousConstants.SPIKE_MARK_WIDTH_IN / 3)
                        .build()
        );
    }
}
