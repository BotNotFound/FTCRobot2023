package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.GearRatio;
import org.firstinspires.ftc.teamcode.modules.concurrent.ConcurrentModule;
import org.firstinspires.ftc.teamcode.modules.concurrent.ModuleThread;

import java.util.concurrent.atomic.AtomicBoolean;

public final class Arm extends ConcurrentModule {
    /**
     * One full rotation of the arm motor in encoder ticks.<br />
     * Taken from <a href="https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-50-9-1-ratio-24mm-length-8mm-rex-shaft-117-rpm-3-3-5v-encoder/">GoBilda</a>
     */
    public static final double ENCODER_RESOLUTION = ((((1+(46.0/17))) * (1+(46.0/17))) * (1+(46.0/17)) * 28);

    public static final GearRatio GEAR_RATIO_EXTERNAL = new GearRatio(20, 100);

    public static final double ONE_REVOLUTION_ENCODER_TICKS =
            GEAR_RATIO_EXTERNAL.calculateStart(ENCODER_RESOLUTION);

    /**
     * The unit of rotation used by default
     */
    public static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;

    /**
     * One full rotation in the unit specified by {@link #ANGLE_UNIT}
     */
    public static final double ONE_REVOLUTION_OUR_ANGLE_UNIT = ANGLE_UNIT.getUnnormalized().fromDegrees(360.0);

    public static final class ArmPresets extends Presets {
        /**
         * Rotates the arm to the position it was in at the start of execution.  This should be parallel to the ground,
         *  with the end of the arm closest to the active intake.
         */
        public static final double START_POS = 0.0;

        /**
         * Rotates the arm so that the robot can collect pixels
         */
        public static final double READY_TO_INTAKE = -25.0;

        /**
         * Rotates the arm so that the robot can deposit pixels on the floor behind the active intake
         */
        public static final double DEPOSIT_ON_FLOOR = 180.0;

        /**
         * Rotates the arm so that the robot can deposit pixels on the backdrop behind the active intake
         */
        public static final double DEPOSIT_ON_BACKDROP = 135.0;
    }

    public static final class WristPresets extends Presets { // TODO these presets are untested
        /**
         * Rotates the wrist so that the robot can collect pixels
         */
        public static final double READY_TO_INTAKE = 30.0;

        /**
         * Rotates the wrist so that the robot can deposit pixels on the floor behind the active intake
         */
        public static final double DEPOSIT_ON_FLOOR = 180.0;

        /**
         * Rotates the wrist so that the robot can deposit pixels on the backdrop behind the active intake
         */
        public static final double DEPOSIT_ON_BACKDROP = 90.0;
    }

    private boolean isFlapOpen;

    /**
     * The motor that rotates the arm
     */
    private final ConditionalHardwareDevice<DcMotor> armMotor;

    /**
     * The name of the arm motor on the hardware map
     */
    public static final String ARM_MOTOR_NAME = "Arm Motor";

    /**
     * The servo that rotates the wrist
     */
    private final ConditionalHardwareDevice<Servo> wristServo;

    /**
     * The name of the wrist servo on the hardware map
     */
    public static final String WRIST_SERVO_NAME = "Wrist Servo";

    /**
     * The servo that opens/closes the flap
     */
    private final ConditionalHardwareDevice<Servo> flapServo;

    /**
     * The name of the flap servo on the hardware map
     */
    public static final String FLAP_SERVO_NAME = "Flap Servo";

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public Arm(OpMode registrar) {
        super(registrar, "Arm Module Threads");
        armMotor = ConditionalHardwareDevice.tryGetHardwareDevice(parent.hardwareMap, DcMotor.class, ARM_MOTOR_NAME);
        wristServo = ConditionalHardwareDevice.tryGetHardwareDevice(parent.hardwareMap, Servo.class, WRIST_SERVO_NAME);
        flapServo = ConditionalHardwareDevice.tryGetHardwareDevice(parent.hardwareMap, Servo.class, FLAP_SERVO_NAME);

        // status update
        armMotor.runIfAvailable(
                device -> getTelemetry().addLine("[Arm] found arm motor of type " + device.getDeviceName() + " on port " + device.getPortNumber()),
                () -> getTelemetry().addLine("[Arm] could not find arm motor!")
        );
        wristServo.runIfAvailable(
                device -> getTelemetry().addLine("[Arm] found wrist servo of type " + device.getDeviceName() + " on port " + device.getPortNumber()),
                () -> getTelemetry().addLine("[Arm] could not find wrist servo!")
        );
        flapServo.runIfAvailable(
                device -> getTelemetry().addLine("[Arm] found flap servo of type " + device.getDeviceName() + " on port " + device.getPortNumber()),
                () -> getTelemetry().addLine("[Arm] could not find flap servo!")
        );

        armMotor.runIfAvailable((arm) -> {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });
        armData = new ArmData();

        isFlapOpen = true;
        closeFlap();

        exitSetup();

    }

    /**
     * A thread that keeps the arm motor at the target position
     */
    @Config("Arm (Position Updater Thread)")
    private static class ArmPositionUpdaterThread extends ModuleThread<Arm> {
        public static final String THREAD_NAME = "Arm Position Updater";

        public static double kP = 0.02;
        public static double kI = 0.0003;
        public static double kD = 0.01;

        /**
         * Initializes the thread
         * @param arm The arm to use
         */
        public ArmPositionUpdaterThread(Arm arm) {
            super(arm, THREAD_NAME);
        }

        @Override
        public void execute() {
            while (host.getState().isInInit()) {
                if (host.getState().isTerminated()) {
                    return; // if OpMode ends in init, end the thread
                }
                Thread.yield(); // wait until OpMode starts before moving the motor
            }

            host.armMotor.runIfAvailable(arm -> { // this thread does nothing if there is no arm motor to update
                int curTarget = 0;
                int error,
                        prevError = 0,
                        errorChange,
                        errorTotal = 0;
                double power;

                while (host.getState().isRunning()) {
                    if (host.armData.isDirty.compareAndSet(true, false)) {
                        curTarget = host.armData.getTargetPosition();
                    }
                    error = arm.getCurrentPosition() - curTarget;
                    errorChange = error - prevError;
                    errorTotal += error;
                    errorTotal = (int)(Math.min(Math.abs(errorTotal), 1 / kI) * Math.signum(errorTotal)); // integral sum limit

                    power = error == 0 ? 0 : (error * kP) + (errorChange * kD) + (errorTotal * kI);
                    arm.setPower(power);
                    prevError = error;
                }
                arm.setPower(0.0); // this probably does something
            });
        }
    }

    private static class ArmData {
        public final AtomicBoolean isDirty = new AtomicBoolean(true);
        private int targetPosition = 0;
        private final Object dataLock = new Object(); // I'm pretty sure this is how threads work
        public int getTargetPosition() {
            synchronized (dataLock) {
                return targetPosition;
            }
        }
        public void setTargetPosition(int newTarget) {
            if (targetPosition == newTarget) {
                return; // nothing to update
            }

            synchronized (dataLock) {
                targetPosition = newTarget;
                isDirty.set(true);
            }
        }
    }
    private final ArmData armData;

    /**
     * Gets the arm motor's internal position
     * @return The arm's position, in encoder ticks
     */
    public int getArmMotorPosition() {
        return armMotor.requireDevice().getCurrentPosition();
    }

    /**
     * Gets the arm motor's target position
     * @return The arm's target position, in encoder ticks
     */
    public int getArmMotorTarget() {
        return armData.getTargetPosition();
    }

    /**
     * Rotates the arm to the specified rotation
     * @param rotation The target rotation
     * @param angleUnit The unit of rotation used
     * @param preserveWristRotation should the wrist rotate with the arm so that it is facing the same direction at the end of rotation?
     */
    public void rotateArmTo(double rotation, AngleUnit angleUnit, boolean preserveWristRotation) {
        if (preserveWristRotation) {
            throw new UnsupportedOperationException("Not Implemented!"); // TODO
        }

        final double normalizedAngle = normalizeAngleOurWay(rotation, angleUnit);
        if (normalizedAngle > ONE_REVOLUTION_OUR_ANGLE_UNIT / 2) {
            return; // don't rotate the arm into the floor
        }

        armData.setTargetPosition((int)Math.round(
                    normalizedAngle
                        * ONE_REVOLUTION_ENCODER_TICKS // multiply before dividing to retain maximum precision
                        / ONE_REVOLUTION_OUR_ANGLE_UNIT
        ));
    }

    /**
     * Normalizes the given angle so that it is within one positive rotation (0-360 degrees, or 0-2pi radians)
     * @param angle The angle to normalize
     * @param unitUsed The unit of the given angle
     * @return The normalized angle
     */
    public static double normalizeAngleOurWay(double angle, AngleUnit unitUsed) {
        angle = ANGLE_UNIT.getUnnormalized().fromUnit(unitUsed.getUnnormalized(), angle);

        angle = angle % ONE_REVOLUTION_OUR_ANGLE_UNIT; // normalize from no rotation up to 1 full revolution
        if (angle < 0) {
            angle += ONE_REVOLUTION_OUR_ANGLE_UNIT; // ensure everything is positive so the arm never rotates into the robot
        }

        return angle;
    }

    /**
     * Rotates the arm to the specified rotation, WITHOUT preserving wrist rotation.
     * @param rotation The target rotation
     * @param angleUnit The unit of rotation used
     */
    public void rotateArmTo(double rotation, AngleUnit angleUnit) {
        rotateArmTo(rotation, angleUnit, false);
    }

    /**
     * Rotates the wrist to the specified rotation
     * @param rotation The target rotation
     * @param angleUnit The unit of rotation used
     */
    public void rotateWristTo(double rotation, AngleUnit angleUnit) {
        wristServo.runIfAvailable((Servo wrist) -> {
            final double convertedRotation = ANGLE_UNIT.fromUnit(angleUnit, rotation); // convert angle to our unit

            throw new RuntimeException("Not implemented!"); // TODO
        });
    }

    /**
     * Gets the rotation of the arm
     * @return The arm's rotation in the unit specified by {@link #ANGLE_UNIT}
     */
    public double getArmRotation() {
        return ((double)getArmMotorPosition() * ONE_REVOLUTION_OUR_ANGLE_UNIT / ONE_REVOLUTION_ENCODER_TICKS);
    }

    /**
     * Gets the rotation of the arm
     * @param angleUnit The unit of rotation to use
     * @return The arm's rotation in the unit specified
     */
    public double getArmRotation(AngleUnit angleUnit) {
        return angleUnit.fromUnit(ANGLE_UNIT, getArmRotation());
    }

    /**
     * Gets the rotation of the wrist
     * @return the rotation in the unit specified by {@link #ANGLE_UNIT}
     */
    public double getWristRotation() {
        return wristServo.requireDevice().getPosition() * (ONE_REVOLUTION_OUR_ANGLE_UNIT / 2); // servo can only rotate up to 180 degrees (1/2 of a full rotation)
    }

    /**
     * Opens the flap, if the flap is not already open
     */
    public void openFlap() {
        flapServo.runIfAvailable((Servo flap) -> {
            if (isFlapOpen) {
                return;
            }
            isFlapOpen = true;

            throw new RuntimeException("Not implemented!"); // TODO
        });
    }

    /**
     * Closes the flap, if the flap is not already closed
     */
    public void closeFlap() {
        flapServo.runIfAvailable((Servo flap) -> {
            if (!isFlapOpen) {
                return;
            }
            isFlapOpen = false;

            throw new RuntimeException("Not implemented!");
        });
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

    public boolean isFlapOpen() {
        flapServo.requireDevice();
        return isFlapOpen;
    }

    @Override
    protected void registerModuleThreads() {
        registerAsyncOperation(new ArmPositionUpdaterThread(this));
    }

    @Override
    public void log() {
        getTelemetry().addData("[Arm] module state", getState());
        armMotor.runIfAvailable(arm -> getTelemetry().addData("[Arm] (arm motor) current rotation", getArmRotation(AngleUnit.DEGREES)));
        wristServo.runIfAvailable(wrist -> getTelemetry().addData("[Arm] (wrist servo) current rotation", getWristRotation()));
        flapServo.runIfAvailable(flap -> getTelemetry().addData("[Arm] is the flap open", isFlapOpen()));
    }
}
