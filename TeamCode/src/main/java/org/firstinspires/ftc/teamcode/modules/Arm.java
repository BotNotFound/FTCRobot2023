package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@Config
public final class Arm extends LinearSlide {

    public static final double ENCODER_RESOLUTION = ((((1+(46.0/17))) * (1+(46.0/17))) * (1+(46.0/17)) * 28);

    public static final double ONE_REVOLUTION_DEGREES = 360;

    private final DcMotorEx jointMotor;

    public static final String JOINT_MOTOR_NAME = "Joint Motor";

    private final PIDFController controller;

    public static double kP = 0,
                        kI = 0,
                        kD = 0,
                        kF = 0;

    private final AtomicInteger targetPosTicks = new AtomicInteger(0);

    private final AtomicBoolean usePIDFLoop = new AtomicBoolean(false);

    public void setUsePIDFLoop(boolean use) {
        usePIDFLoop.set(use);
    }
    public boolean isUsingPIDFLoop() {
        return usePIDFLoop.get();
    }

    private final Timer updateLoop;

    public static abstract class Presets {
        public static final double READY_FOR_INTAKE = 208.0;
        
        public static final double READY_FOR_SCORE = 144.0;

        public static final double IDLE = 5.0;
    }

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public Arm(@NonNull OpMode registrar) throws InterruptedException {
        super(registrar);
        try {
            jointMotor = parent.hardwareMap.get(DcMotorEx.class, JOINT_MOTOR_NAME);
        }
        catch (IllegalArgumentException e) {
            throw new InterruptedException(e.getMessage());
        }

        controller = new PIDFController(kP, kI, kD, kF);

        jointMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        updateLoop = new Timer("Arm PIDF update loop", true);
        TimerTask updatePIDFTask = new TimerTask() {
            @Override
            public void run() {
                if (!usePIDFLoop.get()) { return; }

                double power;
                synchronized (controller) {
                    controller.setPIDF(kP, kI, kD, kF);
                    power = controller.calculate(jointMotor.getCurrentPosition(), targetPosTicks.get());
                }
                jointMotor.setPower(power);
            }
        };
        updateLoop.scheduleAtFixedRate(updatePIDFTask, 0, 10);
    }

    @Override
    public void log() {
        getTelemetry().addData("[Arm] current rotation", getRotation());
        getTelemetry().addData("[Arm] target rotation", (double)(targetPosTicks.get()) / ENCODER_RESOLUTION * ONE_REVOLUTION_DEGREES);
    }

    public double getRotation() {
        return (double)(jointMotor.getCurrentPosition()) * ONE_REVOLUTION_DEGREES / ENCODER_RESOLUTION;
    }

    public void rotateJoint(double rotation) {
        if (usePIDFLoop.get()) {
            targetPosTicks.set((int)(rotation / ONE_REVOLUTION_DEGREES * ENCODER_RESOLUTION));
        }
        else {
            jointMotor.setPower(rotation * 0.4);
        }
    }

    @Override
    public void cleanupModule() {
        super.cleanupModule();
        updateLoop.cancel();
        updateLoop.purge();
    }
}
