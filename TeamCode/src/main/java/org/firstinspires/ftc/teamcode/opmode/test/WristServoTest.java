package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.modules.Arm;

@Autonomous(group = "Tests")
public class WristServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo wrist = hardwareMap.get(Servo.class, Arm.WRIST_SERVO_NAME);
        wrist.setPosition(0.5);
        waitForStart();
        while (opModeIsActive()) {
            wrist.setPosition(0);
            wrist.setPosition(1);
        }
    }
}
