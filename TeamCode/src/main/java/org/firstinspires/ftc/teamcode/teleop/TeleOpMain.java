package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.OpBase;
import org.firstinspires.ftc.teamcode.modules.Arm;

@TeleOp(name="Manual Control")
public class TeleOpMain extends OpBase {

    protected Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;

    @Override
    public void start() {
        super.start();

        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
    }

    @Override
    public void loop() {
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
                gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        // 2nd gamepad controls grabbing
        grabber.rotate(gamepad2.left_stick_y * 0.005);
        arm.setRotation((int)(gamepad2.right_stick_y * Arm.ENCODER_RESOLUTION) + arm.getRotation());
        if (currentGamepad2.a && !previousGamepad2.a) {
            grabber.toggleGrabState();
        }
    }
    
}