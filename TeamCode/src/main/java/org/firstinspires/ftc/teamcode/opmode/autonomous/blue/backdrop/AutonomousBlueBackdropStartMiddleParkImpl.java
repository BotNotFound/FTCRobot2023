package org.firstinspires.ftc.teamcode.autonomous.blue.backdrop;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.autonomous.template.AutonomousConstants;
import org.firstinspires.ftc.teamcode.modules.location.AprilTagLocator;

@Autonomous(name = AutonomousBlueBackdropStartImpl.BLUE_BACKDROP_AUTO_GROUP_NAME + " | Middle", group = AutonomousBlueBackdropStartImpl.BLUE_BACKDROP_AUTO_GROUP_NAME)
public final class AutonomousBlueBackdropStartMiddleParkImpl extends AutonomousBlueBackdropStartImpl {

    /**
     * Called when the robot has finished scoring on the backdrop.  When implemented, parks the robot either to the left
     * or right of the backdrop.
     */
    @Override
    protected void park() {
        final TrajectoryBuilder builder = getDriverToPosition().trajectoryBuilder(getDriverToPosition().getPoseEstimate());
        final AprilTagLocator aprilTagLocator = getModuleManager().getModule(AprilTagLocator.class);

        if (aprilTagLocator.getTagId() == getLeftAprilTagId()) {
            builder.strafeRight((AutonomousConstants.BACKDROP_WIDTH / 3) + AutonomousConstants.TILE_SIDE_LENGTH_IN);
        }
        else if (aprilTagLocator.getTagId() == getCenterAprilTagId()) {
            builder.strafeRight(AutonomousConstants.TILE_SIDE_LENGTH_IN);
        }
        else {
            builder.strafeRight(AutonomousConstants.TILE_SIDE_LENGTH_IN - (AutonomousConstants.BACKDROP_WIDTH / 3));
        }

        builder.forward(AutonomousConstants.TILE_SIDE_LENGTH_IN);

        getDriverToPosition().followTrajectory(builder.build());
    }
}
