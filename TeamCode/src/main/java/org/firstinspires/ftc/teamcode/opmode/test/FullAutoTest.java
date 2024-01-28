package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.core.ModuleManager;
import org.firstinspires.ftc.teamcode.modules.detection.PropDetector;
import org.firstinspires.ftc.teamcode.opmode.autonomous.template.AutonomousConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(group = "Tests")
public class FullAutoTest extends LinearOpMode {
    public static final double UNIT_MOVEMENT = AutonomousConstants.TILE_SIDE_LENGTH_IN * 1.25;

    private SampleMecanumDrive drive;
    private Arm arm;
    private PropDetector propDetector;

    private void initHardware() {

        final ModuleManager moduleManager = new ModuleManager(this);
        drive = new SampleMecanumDrive(this);
        arm = moduleManager.getModule(Arm.class);
        propDetector = moduleManager.getModule(PropDetector.class);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        int aprilTagId = 3;

        initHardware();

        waitForStart();

        driveToSpikeMarks();

        // find which tag id we're using
        for (int i = 1; i < 3; i++) {
            if (isTeamPropDetected()) {
                aprilTagId = i;
                scoreOnSpikeMark();
            }
            turnToNextSpikeMark();
        }

        driveToBackdrop();
        driveToAprilTag(aprilTagId);
        scoreOnBackdrop();

        park();
    }

    private TrajectorySequenceBuilder makeTrajectorySequenceBuilder() {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate());
    }

    private void driveToSpikeMarks() {
        drive.followTrajectorySequence(
            makeTrajectorySequenceBuilder()
                    .strafeLeft(UNIT_MOVEMENT)
                    .build()
        );
    }

    private void turnToNextSpikeMark() {
        drive.followTrajectorySequence(
                makeTrajectorySequenceBuilder()
                        .lineTo(new Vector2d(-UNIT_MOVEMENT / 2))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(UNIT_MOVEMENT / 2))
                        .build()
        );
    }

    private void driveToBackdrop() {
        drive.followTrajectorySequence(
                makeTrajectorySequenceBuilder()
                        .back(UNIT_MOVEMENT * 1.5)
                        .build()
        );
    }

    private void park() {
        drive.followTrajectorySequence(
                makeTrajectorySequenceBuilder()
                        .strafeLeft(UNIT_MOVEMENT)
                        .build()
        );
    }

    private boolean isTeamPropDetected() {
        return false; // TODO
    }

    private void scoreOnSpikeMark() {
        // TODO
    }

    private void driveToAprilTag(int aprilTagId) {
        /*
         TODO if tagId is 1, drive to the left side of the backdrop
                if tagId is 3, drive to the right side of the backdrop
                otherwise, do nothing
        */
    }

    private void scoreOnBackdrop() {
        // TODO
    }
}
