package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.modules.ActiveIntake;
import org.firstinspires.ftc.teamcode.modules.core.ModuleManager;

@TeleOp(group = "Tests")
public class ActiveIntakeTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ModuleManager manager = new ModuleManager(this);
        ActiveIntake in = manager.getModule(ActiveIntake.class);

        waitForStart();

        in.start();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                in.setPower(gamepad1.left_stick_y);
                telemetry.addData("Current Power", in.getPower());
                telemetry.update();
            }
        }
        in.stop();
    }
}
