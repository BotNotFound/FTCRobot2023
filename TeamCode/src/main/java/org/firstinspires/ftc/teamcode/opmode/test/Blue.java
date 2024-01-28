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

import kotlin.Unit;

@Autonomous(group = "Tests", name="Blue Backdrop")
public class Blue extends LinearOpMode {
    public static final double UNIT_MOVEMENT = AutonomousConstants.TILE_SIDE_LENGTH_IN * 1.25;

    protected SampleMecanumDrive drive;
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

        if (aprilTagId == 3) {
            scoreOnSpikeMark();
        }

        driveToBackdrop();
        driveToAprilTag(aprilTagId);
        scoreOnBackdrop();

        park(aprilTagId);
    }

    protected TrajectorySequenceBuilder makeTrajectorySequenceBuilder() {
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

    protected void driveToBackdrop() {
        drive.followTrajectorySequence(
                makeTrajectorySequenceBuilder()
                        .strafeLeft(UNIT_MOVEMENT / 2)
                        .build()
        );
        drive.followTrajectorySequence(
                makeTrajectorySequenceBuilder()
                        .back(UNIT_MOVEMENT * 1.5)
                        .build()
        );
        drive.followTrajectorySequence(
                makeTrajectorySequenceBuilder()
                        .strafeRight(UNIT_MOVEMENT / 2)
                        .build()
        );
    }

    protected void park(int aprilTagId) {
        double tempMovement = UNIT_MOVEMENT;
        if (aprilTagId == 1) {
            tempMovement += UNIT_MOVEMENT / 4;
        } else if (aprilTagId == 3) {
            tempMovement -= UNIT_MOVEMENT / 4;
        }
        drive.followTrajectorySequence(
                makeTrajectorySequenceBuilder()
                        .strafeLeft(tempMovement)
                        .build()
        );
    }

    private boolean isTeamPropDetected() {
        return propDetector.isPropDetected();
    }

    private void scoreOnSpikeMark() {
        arm.rotateArmAndWrist(Arm.ArmPresets.DEPOSIT_ON_FLOOR, Arm.WristPresets.DEPOSIT_ON_FLOOR);
        arm.cycleFlap();
    }

    private void driveToAprilTag(int aprilTagId) {
        if (aprilTagId == 1) {
            drive.followTrajectorySequence(
                    makeTrajectorySequenceBuilder()
                            .strafeLeft(UNIT_MOVEMENT / 4)
                            .build()
            );
        } else if (aprilTagId == 3) {
            drive.followTrajectorySequence(
                    makeTrajectorySequenceBuilder()
                            .strafeRight(UNIT_MOVEMENT / 4)
                            .build()
            );
        }
    }

    private void scoreOnBackdrop() {
        arm.rotateArmAndWrist(Arm.ArmPresets.DEPOSIT_ON_BACKDROP, Arm.WristPresets.DEPOSIT_ON_BACKDROP);
        arm.cycleFlap();
    }
}
