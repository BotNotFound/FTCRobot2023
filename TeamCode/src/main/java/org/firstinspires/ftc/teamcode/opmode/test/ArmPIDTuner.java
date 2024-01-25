package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.modules.Arm;

import static org.firstinspires.ftc.teamcode.modules.Arm.*;

@TeleOp(group = "Tests")
public final class ArmPIDTuner extends OpMode {

    private Gamepad currentGamepad1, previousGamepad1;
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

    private boolean checkFailsafe() {
        if (failsafeEngaged) {
            return true; // no need to terminate multiple times
        }

        if (gamepad1.a || gamepad1.back) {
            failsafeEngaged = true;
            arm.interrupt();
            terminateOpModeNow(); // failsafe
            arm.cleanupModule();
            return true;
        }

        return false;
    }

    private boolean failsafeEngaged;

    private Arm arm;

    @Override
    public void init() {
        failsafeEngaged = false;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = new Arm(this);
    }

    @Override
    public void init_loop() {
        if (checkFailsafe()) return;

        telemetry.addData("curPos", 0);
        telemetry.addData("tPos", 0);
        telemetry.addData("s", arm.getState());
    }

    @Override
    public void start() {
        super.start();
        arm.startThreads();

        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();
    }

    @Override
    public void loop() {
        if (checkFailsafe()) return;

        // Store the gamepad values from the previous loop iteration in
        // previousGamepad1/2 to be used in this loop iteration.
        // This is equivalent to doing this at the end of the previous
        // loop iteration, as it will run in the same order except for
        // the first/last iteration of the loop.
        previousGamepad1.copy(currentGamepad1);
        //previousGamepad2.copy(currentGamepad2);

        // Store the gamepad values from this loop iteration in
        // currentGamepad1/2 to be used for the entirety of this loop iteration.
        // This prevents the gamepad values from changing between being
        // used and stored in previousGamepad1/2.
        currentGamepad1.copy(gamepad1);
        //currentGamepad2.copy(gamepad2);

        if (gamepad1.y) {
            ArmTestConfig.targetPosition = ArmTestConfig.angleUnit.fromDegrees(90);
            arm.rotateArmTo((-gamepad1.left_stick_y * 90) + 90, AngleUnit.DEGREES); // [-90, 90] + 90 = [0, 180]
        } else {
            arm.rotateArmTo(ArmTestConfig.targetPosition, ArmTestConfig.angleUnit);
        }

        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            arm.cycleFlap();
        }
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            arm.fullCycleFlap();
        }

        telemetry.addData("curPos", arm.getArmMotorPosition());
        telemetry.addData("tPos", arm.getArmMotorTarget());
        telemetry.addData("s", arm.getState());
    }

    @Override
    public void stop() {
        super.stop();
        arm.cleanupModule();
    }
}
