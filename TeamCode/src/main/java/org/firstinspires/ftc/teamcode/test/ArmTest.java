package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.modules.Arm;

import static org.firstinspires.ftc.teamcode.modules.Arm.*;

@TeleOp(group = "Tests")
public final class ArmTest extends OpMode {
    @Disabled
    @TeleOp(group = "Tests")
    public static class AT2 extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            waitForStart();

            double input, normalized, output;
            while (opModeIsActive()) {
                input = gamepad1.left_stick_y;
                input *= 180;
                telemetry.addData("input", input);

                normalized = normalizeAngleOurWay(input, AngleUnit.DEGREES);
                telemetry.addData("normalized", normalized);

                output = GEAR_RATIO_EXTERNAL.calculateStart(normalized) * ENCODER_RESOLUTION // multiply before dividing to retain maximum precision
                        / ONE_REVOLUTION_OUR_ANGLE_UNIT;
                telemetry.addData("output", output);

                telemetry.update();
            }
        }
    }

    @Config("Arm Test")
    public static class ArmTestConfig {
        public static double targetPosition = 90;
        public static AngleUnit angleUnit = AngleUnit.DEGREES;
    }

    private Arm arm;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = new Arm(this);
    }

    @Override
    public void init_loop() {
        telemetry.addData("curPos", 0);
        telemetry.addData("tPos", 0);
        telemetry.addData("s", arm.getArmThreadState());
    }

    @Override
    public void start() {
        super.start();
        arm.startModule();
    }

    @Override
    public void loop() {
        if (gamepad1.a || gamepad1.back) {
            arm.pleaseStopTheArmThreadBeforeSomethingBreaks();
            terminateOpModeNow(); // failsafe
            arm.cleanupModule();
            return;
        }

        if (gamepad1.y) {
            ArmTestConfig.targetPosition = ArmTestConfig.angleUnit.fromDegrees(90);
            arm.rotateArmTo((-gamepad1.left_stick_y * 90) + 90, AngleUnit.DEGREES); // [-90, 90] + 90 = [0, 180]
        } else {
            arm.rotateArmTo(ArmTestConfig.targetPosition, ArmTestConfig.angleUnit);
        }

        telemetry.addData("curPos", arm.getArmMotorPosition());
        telemetry.addData("tPos", arm.getArmMotorTarget());
        telemetry.addData("s", arm.getArmThreadState());
    }

    @Override
    public void stop() {
        super.stop();
        arm.cleanupModule();
    }
}
