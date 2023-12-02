package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.OpBase;
import org.firstinspires.ftc.teamcode.modules.Arm;

import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(name="Manual Control")
public final class TeleOpMain extends OpBase {

    private boolean checkFailsafe() {
        if (gamepad1.guide || gamepad1.ps || gamepad2.guide || gamepad2.ps) {
            terminateOpModeNow();
            return true;
        }
        return false;
    }

    private Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;

    @Override
    public void init_loop() {
        super.init_loop();
        checkFailsafe();
    }

    @Override
    public void start() {
        super.start();
        driveTrain.resetRotation();

        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
    }
    private final AtomicBoolean launchedPlane = new AtomicBoolean(false);

    @Override
    public void loop() {
        if (checkFailsafe()) return;

        // Store the gamepad values from the previous loop iteration in
        // previousGamepad1/2 to be used in this loop iteration.
        // This is equivalent to doing this at the end of the previous
        // loop iteration, as it will run in the same order except for
        // the first/last iteration of the loop.
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        // Store the gamepad values from this loop iteration in
        // currentGamepad1/2 to be used for the entirety of this loop iteration.
        // This prevents the gamepad values from changing between being
        // used and stored in previousGamepad1/2.
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);


        // 1st gamepad controls movement
        driveTrain.setVelocity(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x * 0.5
        );
        if (currentGamepad1.start) {
            driveTrain.resetRotation();
        }

        // 2nd gamepad controls grabbing and plane launcher
        if (currentGamepad2.y && launchedPlane.compareAndSet(false, true)) {
            planeLauncher.launch();
        }
        // convert gamepad range of [-1,1] to extendTo()'s range of [0,1]
//        arm.extendTo(Math.max(gamepad2.right_stick_y, 0));

        if (arm.isUsingPIDFLoop()) {
            if (gamepad2.dpad_left) {
                arm.rotateJoint(Arm.Presets.IDLE);
            } else if (gamepad2.dpad_up) {
                arm.rotateJoint(Arm.Presets.READY_FOR_SCORE);
            } else if (gamepad2.dpad_right) {
                arm.rotateJoint(Arm.Presets.READY_FOR_INTAKE);
            }
        }
        else if (Math.abs(gamepad2.left_stick_y) < 0.1) {
            arm.rotateJoint(Math.cos((arm.getRotation() / 180 * Math.PI) - (Math.PI / 6)) * 0.2);
        }
        else {
            arm.rotateJoint(-(gamepad2.left_stick_y));
        }
//        if (currentGamepad2.back && !previousGamepad2.back) {
//            arm.setUsePIDFLoop(!arm.isUsingPIDFLoop());
//        }

        // preset grabber positions
        if (currentGamepad2.a && !previousGamepad2.a) {
            doubleClaw.incrementClawState();
        }
    }
    
}