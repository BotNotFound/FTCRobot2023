package org.firstinspires.ftc.teamcode.opmode.autonomous.template;

import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.detection.Prop;
import org.firstinspires.ftc.teamcode.modules.detection.PropDetector;
import org.firstinspires.ftc.teamcode.opmode.OpBaseLinear;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public abstract class AutonomousBase extends OpBaseLinear {
    /**
     * The prop detector
     */
    private PropDetector propDetector;

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

    private Arm arm;

    protected final int getAprilTagId() {
        return aprilTagId;
    }

    protected final void setAprilTagId(int aprilTagId) {
        this.aprilTagId = aprilTagId;
    }

    private int aprilTagId;

    @Override
    protected void initModules() {
        propDetector = getModuleManager().getModule(PropDetector.class);
        arm = getModuleManager().getModule(Arm.class);
        driverToPosition = new SampleMecanumDrive(this);
    }

    @Override
    public void runOpMode() {
        // assumed starting position is touching the wall, with the active intake facing away from the backdrop

        driveToSpikeMarks();

        // determine what side team prop is on
        final Prop teamProp = getTeamProp();
        if (propDetector.isPropDetected(teamProp)) { // prop detected on the side facing the wall opposite the backdrop
            setAprilTagId(getLeftAprilTagId());
        }
        else {
            rotateToNextSpikeMark();
            if (propDetector.isPropDetected(teamProp)) { // prop detected on the side facing the opposing alliance
                setAprilTagId(getCenterAprilTagId());
            }
            else {
                rotateToNextSpikeMark();
                setAprilTagId(getRightAprilTagId()); // assume prop is on the side facing the backdrop
            }
        }
        scoreOnSpikeMark();

        prepareArmForDriving();
        driveToBackdrop();
        scoreOnBackdrop();

        prepareArmForDriving();
        park();
    }

    /**
     * Called after methods that move the arm, before methods that move the robot.  Moves the arm and wrist so that the
     *  robot will be able to drive under the trusses without getting stuck
     */
    protected final void prepareArmForDriving() {
        arm.closeFlap();
        arm.rotateArmAndWrist(Arm.ArmPresets.IDLE, Arm.WristPresets.IDLE);
    }

    /**
     * Called when the robot has finished scoring on the backdrop.  When implemented, parks the robot either to the left
     *  or right of the backdrop.
     */
    protected abstract void park();

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
        arm.rotateArmAndWrist(Arm.ArmPresets.DEPOSIT_ON_BACKDROP, Arm.WristPresets.DEPOSIT_ON_BACKDROP);
        arm.cycleFlap();
    }

    /**
     * Called at the beginning of execution, when the robot is facing the wall opposite the backdrop and has one side
     *  touching the wall.  When implemented, strafes directly to the U of spike marks.
     */
    protected abstract void driveToSpikeMarks();

    /**
     * @return The ID of the AprilTag on the left of the backdrop
     * @see #driveToSpikeMarks()
     */
    protected abstract int getLeftAprilTagId();

    /**
     * @return The ID of the AprilTag in the middle of the backdrop
     */
    protected abstract int getCenterAprilTagId();

    /**
     * @return The ID of the AprilTag on the right of the backdrop
     * @see #driveToSpikeMarks()
     */
    protected abstract int getRightAprilTagId();

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
