package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.OpBaseLinear;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.detection.Prop;
import org.firstinspires.ftc.teamcode.modules.detection.PropDetector;
import org.firstinspires.ftc.teamcode.modules.location.AprilTagLocator;
import org.firstinspires.ftc.teamcode.modules.location.LocalizedMovement;
import org.firstinspires.ftc.teamcode.modules.location.Odometry;

@Autonomous(name = "Autonomous Main (Drive To Position)")
public class AutonomousMainReal extends OpBaseLinear {
    /**
     * Used to get where to place the 2nd pixel
     */
    private AprilTagLocator aprilTagLocator;

    /**
     * The distance to the '|_|' shape where the team prop will be
     */
    public static final double DISTANCE_TO_TEAM_PROP_U = 50.0;

    /**
     * The prop detector
     */
    private PropDetector propDetector;

    public static final Prop teamProp = Prop.BLUE_TEAM_PROP;

    public static final int LEFT_APRIL_TAG_ID = 4;
    public static final int CENTER_APRIL_TAG_ID = 5;
    public static final int RIGHT_APRIL_TAG_ID = 6;

    @Override
    public void initHardware() throws InterruptedException {
        super.initHardware();
        aprilTagLocator = new AprilTagLocator(this);
        propDetector = new PropDetector(this);
    }

    @Override
    public void runOpMode() {
        // assumed starting position is touching the wall, with the active intake facing away from the backdrop

        driveTrain.driveToOdometry(Movement.Axis.Y.genPointFromAxis(DISTANCE_TO_TEAM_PROP_U)); // drive sideways to the U

        // determine what side team prop is on
        if (propDetector.isPropDetected(teamProp)) { // prop detected on the side facing the wall opposite the backdrop
            aprilTagLocator.setTagId(LEFT_APRIL_TAG_ID);
        }
        else {
            driveTrain.driveToOdometry(driveTrain.getLocation().add(Movement.Axis.THETA.genPointFromAxis(Odometry.ANGLE_UNIT.fromDegrees(-90))));
            if (propDetector.isPropDetected(teamProp)) { // prop detected on the side facing the opposing alliance
                aprilTagLocator.setTagId(CENTER_APRIL_TAG_ID);
            }
            else {
                driveTrain.driveToOdometry(driveTrain.getLocation().add(Movement.Axis.THETA.genPointFromAxis(Odometry.ANGLE_UNIT.fromDegrees(-90))));
                aprilTagLocator.setTagId(RIGHT_APRIL_TAG_ID); // assume prop is on the side facing the backdrop
            }
        }
        scoreOnSpikeMark();


        // use location of team prop to determine path


        driveTrain.driveToOdometry(driveTrain.getLocation().add(Movement.Axis.THETA.genPointFromAxis(Odometry.ANGLE_UNIT.fromDegrees(90)))); // rotate so we can see the AprilTags
        // AprilTagLocator only works if we can see the April Tag, So we convert its location to a more reliable locator
        LocalizedMovement aprilTagLoc_odom = new LocalizedMovement(0, 1, aprilTagLocator).convertToOtherLocator(driveTrain);
        driveTrain.driveTo(aprilTagLoc_odom);

        scoreOnBackdrop();

        park();
    }

    @Override
    public void stop() {
        super.stop();
        aprilTagLocator.cleanupModule();
    }
    
    public void park() {} // TODO finish

    public void scoreOnSpikeMark() {} // TODO finish

    public void scoreOnBackdrop() {} // TODO finish
}
