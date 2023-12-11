package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.jetbrains.annotations.NotNull;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@Config
public final class Arm extends ModuleBase {
    public static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;

    private boolean isFlapOpen;

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public Arm(OpMode registrar) throws InterruptedException {
        super(registrar);
        isFlapOpen = true;
        closeFlap();
    }

    /**
     * Rotates the arm to the specified rotation
     * @param rotation The target rotation
     * @param angleUnit The unit of rotation used
     */
    public void rotateArmTo(double rotation, AngleUnit angleUnit) {
        rotation = ANGLE_UNIT.fromUnit(angleUnit, rotation); // convert angle to our unit

    }

    /**
     * Rotates the wrist to the specified rotation
     * @param rotation The target rotation
     * @param angleUnit The unit of rotation used
     */
    public void rotateWristTo(double rotation, AngleUnit angleUnit) {
        rotation = ANGLE_UNIT.fromUnit(angleUnit, rotation); // convert angle to our unit

    }

    /**
     * Opens the flap, if the flap is not already open
     */
    public void openFlap() {
        if (isFlapOpen) {
            return;
        }
        isFlapOpen = false;

    }

    /**
     * Closes the flap, if the flap is not already closed
     */
    public void closeFlap() {
        if (!isFlapOpen) {
            return;
        }
        isFlapOpen = true;

    }

    /**
     * If the flap is open, close it.  Otherwise, open the flap
     */
    public void toggleFlap() {
        if (isFlapOpen) {
            closeFlap();
        }
        else {
            openFlap();
        }
    }

    @Override
    public void cleanupModule() {

    }

    @Override
    public void log() {

    }
}
