package org.firstinspires.ftc.teamcode.opmode.autonomous.red.wall;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.modules.location.AprilTagLocator;
import org.firstinspires.ftc.teamcode.opmode.autonomous.template.AutonomousConstants;

@Autonomous(name = AutonomousRedWallStartImpl.RED_WALL_AUTO_GROUP_NAME + " | Middle", group = AutonomousRedWallStartImpl.RED_WALL_AUTO_GROUP_NAME)
public final class AutonomousRedWallStartMiddleParkImpl extends AutonomousRedWallStartImpl {

    /**
     * Called when the robot has finished scoring on the backdrop.  When implemented, parks the robot either to the left
     * or right of the backdrop.
     */
    @Override
    protected void park() {
        final TrajectoryBuilder builder = getDriverToPosition().trajectoryBuilder(getDriverToPosition().getPoseEstimate());
        final AprilTagLocator aprilTagLocator = getModuleManager().getModule(AprilTagLocator.class);

        if (aprilTagLocator.getTagId() == getRightAprilTagId()) {
            builder.strafeLeft((AutonomousConstants.BACKDROP_WIDTH / 3) + AutonomousConstants.TILE_SIDE_LENGTH_IN);
        }
        else if (aprilTagLocator.getTagId() == getCenterAprilTagId()) {
            builder.strafeLeft(AutonomousConstants.TILE_SIDE_LENGTH_IN);
        }
        else {
            builder.strafeLeft(AutonomousConstants.TILE_SIDE_LENGTH_IN - (AutonomousConstants.BACKDROP_WIDTH / 3));
        }

        builder.forward(AutonomousConstants.TILE_SIDE_LENGTH_IN);

        getDriverToPosition().followTrajectory(builder.build());
    }
}
