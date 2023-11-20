package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpBase;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(name="Manual Control")
public final class TeleOpMain extends OpBase {

    private Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;

    private VisionPortal portal;

    @Override
    public void start() {
        super.start();

        /*portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .setCamera(BuiltinCameraDirection.BACK)
                .build();*/

        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
    }
    AtomicBoolean launchedPlane = new AtomicBoolean(false);

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
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        telemetry.addData("Launched Plane", launchedPlane);

        // 2nd gamepad controls grabbing and plane launcher
        if (currentGamepad2.y && launchedPlane.compareAndSet(false, true)) {
            planeLauncher.launch();
        }
        arm.rotate(gamepad2.right_stick_y);
        if (currentGamepad2.a && !previousGamepad2.a) {
            grabber.toggleGrabState();
        }
        // preset grabber positions
        if (currentGamepad2.left_bumper) {
            grabber.setRotation(0.5);
        }
        else if (currentGamepad2.right_bumper) {
            grabber.setRotation(0.75);
        }
        else if (currentGamepad2.back) {
            grabber.setRotation(0);
        }
        else {
            grabber.rotate(-gamepad2.left_stick_y * 0.005);
        }
    }
    
}