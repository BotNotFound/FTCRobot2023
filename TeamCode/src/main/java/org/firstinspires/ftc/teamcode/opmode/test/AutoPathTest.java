package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.opmode.autonomous.template.AutonomousConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(group = "Tests")
public class AutoPathTest extends LinearOpMode {
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(this);

        final double UNIT_MOVEMENT = AutonomousConstants.TILE_SIDE_LENGTH_IN * 1.25;

        waitForStart();

        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(new Pose2d())
                        // gp to spike marks
                        .strafeLeft(UNIT_MOVEMENT)

                        // turn around
                        .strafeRight(UNIT_MOVEMENT * 0.75)
                        .turn(-Math.PI)
                        .strafeRight(UNIT_MOVEMENT * 0.75)

                        // go to backdrop
                        .back(UNIT_MOVEMENT * 1.5)

                        // park
                        .strafeLeft(UNIT_MOVEMENT)

                        .build()
        );
    }
}
