package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.modules.*;
import org.firstinspires.ftc.teamcode.opmode.OpBase;

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

    private FieldCentricDriveTrain driveTrain;
    private Arm arm;
    private ActiveIntake activeIntake;
    private PlaneLauncher planeLauncher;
//    private HangModule hangModule;

    private ConditionalHardwareDevice<DcMotorEx> hangMotor;

    private double WRIST_TO_ARM_WAIT = 0.5;
    private double TARGET_WRIST_TIME = 0;
    private boolean ROTATE_ARM_IDLE;
    private boolean ROTATE_WRIST_INTAKE;

    @Override
    protected void initModules() {
        driveTrain = getModuleManager().getModule(FieldCentricDriveTrain.class);
        arm = getModuleManager().getModule(Arm.class);
        activeIntake = getModuleManager().getModule(ActiveIntake.class);
        planeLauncher = getModuleManager().getModule(PlaneLauncher.class);
//        hangModule = getModuleManager().getModule(HangModule.class);

        hangMotor = ConditionalHardwareDevice.tryGetHardwareDevice(hardwareMap, DcMotorEx.class, HangModule.HANG_MOTOR_NAME);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        checkFailsafe();
    }

    @Override
    public void start() {
        super.start();
        driveTrain.resetRotation();
        activeIntake.start();

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
                -gamepad1.left_stick_y * 0.5,
                gamepad1.left_stick_x * 0.5,
                gamepad1.right_stick_x * 0.5
        );
        if (currentGamepad1.start && !previousGamepad1.start) {
            driveTrain.resetRotation();
        }

        // 2nd gamepad controls grabbing and plane launcher
        if (currentGamepad2.start) {
            planeLauncher.launch();
        }

        if (gamepad2.x) {
            arm.rotateWristTo(Arm.WristPresets.IDLE);
            startTimer();
            ROTATE_ARM_IDLE = true;

        } else if (gamepad2.y) {
            arm.rotateWristTo(Arm.WristPresets.DEPOSIT_ON_BACKDROP);
            arm.rotateArmTo(Arm.ArmPresets.DEPOSIT_ON_BACKDROP, Arm.ANGLE_UNIT);
        } else if (gamepad2.b) {
            arm.rotateWristTo(Arm.WristPresets.DEPOSIT_ON_FLOOR);
            arm.rotateArmTo(Arm.ArmPresets.DEPOSIT_ON_FLOOR, Arm.ANGLE_UNIT);
        }
        else if (gamepad2.a) {
            arm.rotateArmTo(Arm.ArmPresets.READY_TO_INTAKE, Arm.ANGLE_UNIT);
            startTimer();
            ROTATE_WRIST_INTAKE = true;

        }
        if(ROTATE_ARM_IDLE && getRuntime() > TARGET_WRIST_TIME) {
            arm.rotateArmTo(Arm.ArmPresets.IDLE, Arm.ANGLE_UNIT);
            ROTATE_ARM_IDLE = false;
        }
        if(ROTATE_WRIST_INTAKE && getRuntime() > TARGET_WRIST_TIME) {
            arm.rotateWristTo(Arm.WristPresets.READY_TO_INTAKE);
            ROTATE_WRIST_INTAKE = false;
        }

        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            arm.cycleFlap();
        }
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            arm.fullCycleFlap();
        }

        if (currentGamepad1.b && !previousGamepad1.b) {
            activeIntake.stop();
        }
        else if (currentGamepad1.a && !previousGamepad1.a) {
            activeIntake.start();
        }
        else if (currentGamepad1.right_bumper && !previousGamepad2.right_bumper) {
            activeIntake.reverse();
        } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            activeIntake.turbo();
        } else if (previousGamepad1.left_bumper) {
            activeIntake.unTurbo();
        }

//        if (currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button) {
//            hangModule.toggleHangState();
//        }
        hangMotor.runIfAvailable(motor -> {
            motor.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            telemetry.addData("[Hang] motor amp usage", motor.getCurrent(CurrentUnit.MILLIAMPS));
        });

        getModuleManager().logModuleStatus();
    }

    private void startTimer() {
        TARGET_WRIST_TIME = getRuntime() + WRIST_TO_ARM_WAIT;
    }
    
}