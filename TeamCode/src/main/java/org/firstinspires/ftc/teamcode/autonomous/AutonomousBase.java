package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.OpBaseLinear;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.detection.Prop;
import org.firstinspires.ftc.teamcode.modules.detection.PropDetector;
import org.firstinspires.ftc.teamcode.modules.location.AprilTagLocator;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public abstract class AutonomousBase extends OpBaseLinear {
    /**
     * Used to get where to place the 2nd pixel
     */
    private AprilTagLocator aprilTagLocator;

    /**
     * The distance to the '|_|' shape where the team prop will be (in millimeters)
     */
    public static final double DISTANCE_TO_TEAM_PROP_U_MM = 50.0;

    /**
     * The prop detector
     */
    private PropDetector propDetector;

    /**
     * Gets the prop detector
     * @return the prop detector
     */
    protected final PropDetector getPropDetector() {
        return propDetector;
    }

    /**
     * Roadrunner's drive-to-position drive train
     */
    private SampleMecanumDrive driverToPosition;

    /**
     * Gets the {@link SampleMecanumDrive}
     * @return Roadrunner's drive-to-position drive train
     */
    protected final SampleMecanumDrive getDriverToPosition() {
        return driverToPosition;
    }

    @Override
    public void initHardware() throws InterruptedException {
        super.initHardware();
        aprilTagLocator = new AprilTagLocator(this);
        propDetector = new PropDetector(this);
        driverToPosition = new SampleMecanumDrive(hardwareMap, driveTrain);
    }

    @Override
    public void runOpMode() {
        // assumed starting position is touching the wall, with the active intake facing away from the backdrop

        driveToSpikeMarks();

        // determine what side team prop is on
        final Prop teamProp = getTeamProp();
        if (propDetector.isPropDetected(teamProp)) { // prop detected on the side facing the wall opposite the backdrop
            aprilTagLocator.setTagId(getWallSideAprilTagId());
        }
        else {
            rotateToNextSpikeMark();
            if (propDetector.isPropDetected(teamProp)) { // prop detected on the side facing the opposing alliance
                aprilTagLocator.setTagId(getCenterAprilTagId());
            }
            else {
                rotateToNextSpikeMark();
                aprilTagLocator.setTagId(getOpenSideAprilTagId()); // assume prop is on the side facing the backdrop
            }
        }
        scoreOnSpikeMark();


        driveToBackdrop();
        scoreOnBackdrop();

        park();
    }

    @Override
    public void stop() {
        super.stop();
        aprilTagLocator.cleanupModule();
    }

    /**
     * Called after methods that move the arm, before methods that move the robot.  Moves the arm and wrist so that the
     *  robot will be able to drive under the trusses without getting stuck
     */
    private void prepareArmForDriving() {
        arm.closeFlap();
        arm.rotateArmTo(Arm.ArmPresets.READY_TO_INTAKE);

    }

    /**
     * Called when the robot has finished scoring on the backdrop.  When implemented, parks the robot either to the left
     *  or right of the backdrop.
     */
    public abstract void park();

    /**
     * Called after the target AprilTag is set, when the robot is in the U of spike marks, with the team prop directly
     *  in front of it.  When implemented, moves the robot so that the pixel won't be placed on top of the team prop, and
     *  places the first pixel on the spike mark.
     */
    protected abstract void scoreOnSpikeMark();

    /**
     * Called when the robot is facing the backdrop, directly in front of the target AprilTag.  Places the second pixel
     *  on the backdrop.
     */
    protected void scoreOnBackdrop() {
        arm.rotateArmTo(Arm.ArmPresets.DEPOSIT_ON_BACKDROP);
        arm.rotateWristTo(Arm.WristPresets.DEPOSIT_ON_BACKDROP);
        arm.openFlap();
    }

    /**
     * Called at the beginning of execution, when the robot is facing the wall opposite the backdrop and has one side
     *  touching the wall.  When implemented, strafes directly to the U of spike marks.
     */
    protected abstract void driveToSpikeMarks();

    /**
     * @return The ID of the AprilTag closest to the wall that the robot was touching at the start of execution
     * @see #driveToSpikeMarks()
     */
    protected abstract int getWallSideAprilTagId();

    /**
     * @return The ID of the AprilTag in the middle of the backdrop
     */
    protected abstract int getCenterAprilTagId();

    /**
     * @return The ID of the AprilTag furthest away from the wall that the robot was touching at the start of execution
     * @see #driveToSpikeMarks()
     */
    protected abstract int getOpenSideAprilTagId();

    /**
     * @return A {@link Prop} object representing the team prop for our alliance
     */
    protected abstract Prop getTeamProp();

    /**
     * Called twice, both times in the middle of the U of spike marks, facing a spike mark.  When implemented, moves
     *  outside the U (through the side without a spike mark), rotates 90 degrees to the next spike mark.
     */
    protected abstract void rotateToNextSpikeMark();

    /**
     * Called when the robot is in the middle of the U of spike marks, facing the backdrop.  When implemented, moves the
     *  robot to the backdrop so that it is directly in front of the target AprilTag.
     */
    protected abstract void driveToBackdrop();
}
