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

@Autonomous(group = "Tests", name="Blue Wall")
public class BlueWall extends Blue {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }

    @Override
    protected void driveToBackdrop() {
        drive.followTrajectorySequence(
                makeTrajectorySequenceBuilder()
                        .strafeRight(UNIT_MOVEMENT / 2)
                        .build()
        );
        drive.followTrajectorySequence(
                makeTrajectorySequenceBuilder()
                        .back(UNIT_MOVEMENT * 3.5)
                        .build()
        );
        drive.followTrajectorySequence(
                makeTrajectorySequenceBuilder()
                        .strafeLeft(UNIT_MOVEMENT / 2)
                        .build()
        );
    }

    @Override
    protected void park(int aprilTagId) {
        double tempMovement = UNIT_MOVEMENT;
        if (aprilTagId == 1) {
            tempMovement += UNIT_MOVEMENT / 4;
        } else if (aprilTagId == 3) {
            tempMovement -= UNIT_MOVEMENT / 4;
        }
        drive.followTrajectorySequence(
                makeTrajectorySequenceBuilder()
                        .strafeRight(tempMovement)
                        .build()
        );
    }
}
