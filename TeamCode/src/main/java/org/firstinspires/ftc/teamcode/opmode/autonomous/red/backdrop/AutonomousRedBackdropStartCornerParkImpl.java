package org.firstinspires.ftc.teamcode.opmode.autonomous.red.backdrop;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.opmode.autonomous.template.AutonomousConstants;

@Autonomous(name = AutonomousRedBackdropStartImpl.RED_BACKDROP_AUTO_GROUP_NAME + " | Corner", group = AutonomousRedBackdropStartImpl.RED_BACKDROP_AUTO_GROUP_NAME)
public final class AutonomousRedBackdropStartCornerParkImpl extends AutonomousRedBackdropStartImpl {

    /**
     * Called when the robot has finished scoring on the backdrop.  When implemented, parks the robot either to the left
     * or right of the backdrop.
     */
    @Override
    protected void park() {
        final TrajectoryBuilder builder = getDriverToPosition().trajectoryBuilder(getDriverToPosition().getPoseEstimate());

        if (getAprilTagId() == getLeftAprilTagId()) {
            builder.strafeRight((AutonomousConstants.BACKDROP_WIDTH / 3) + AutonomousConstants.TILE_SIDE_LENGTH_IN);
        }
        else if (getAprilTagId() == getCenterAprilTagId()) {
            builder.strafeRight(AutonomousConstants.TILE_SIDE_LENGTH_IN);
        }
        else {
            builder.strafeRight(AutonomousConstants.TILE_SIDE_LENGTH_IN - (AutonomousConstants.BACKDROP_WIDTH / 3));
        }

        builder.forward(AutonomousConstants.TILE_SIDE_LENGTH_IN);

        getDriverToPosition().followTrajectory(builder.build());
    }
}
