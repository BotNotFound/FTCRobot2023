package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.GearRatio;

import java.util.concurrent.atomic.AtomicBoolean;

public final class Arm extends ModuleBase {
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
        super(registrar);
        armMotor = ConditionalHardwareDevice.tryGetHardwareDevice(parent.hardwareMap, DcMotor.class, ARM_MOTOR_NAME);
        wristServo = ConditionalHardwareDevice.tryGetHardwareDevice(parent.hardwareMap, Servo.class, WRIST_SERVO_NAME);
        flapServo = ConditionalHardwareDevice.tryGetHardwareDevice(parent.hardwareMap, Servo.class, FLAP_SERVO_NAME);

        armMotor.runIfAvailable((arm) -> {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });
        armData = new ArmData();
        armPositionUpdaterThread = new ArmPositionUpdaterThread(this);
        armPositionUpdaterThread.start();

        isFlapOpen = true;
        closeFlap();
    }

    public void pleaseStopTheArmThreadBeforeSomethingBreaks() {
        armPositionUpdaterThread.interrupt();
    }

    public void startModule() {
        armData.startUpdateLoop.set(true);
    }

    /**
     * A thread that keeps the arm motor at the target position
     */
    @Config("Arm (Position Updater Thread)")
    private static class ArmPositionUpdaterThread extends Thread {
        public static final String THREAD_NAME = "Arm Position Updater";

        private final ConditionalHardwareDevice<DcMotor> couldBeArmMotor;
        private final ArmData data;

        public static double kP = 0.02;
        public static double kI = 0.0003;
        public static double kD = 0.01;

        /**
         * Initializes the thread
         * @param arm The arm to use
         */
        public ArmPositionUpdaterThread(Arm arm) {
            super(THREAD_NAME);
            couldBeArmMotor = arm.armMotor;
            data = arm.armData;
        }

        @Override
        public void run() {
            couldBeArmMotor.runIfAvailable(arm -> { // this thread does nothing if there is no arm motor to update
                while (!data.startUpdateLoop.get()) {
                    if (!data.keepThreadAlive.get()) {
                        return; // if OpMode ends in init, end the thread
                    }
                    Thread.yield(); // wait until OpMode starts before moving the motor
                }

                int curTarget = 0;
                int error,
                        prevError = 0,
                        errorChange,
                        errorTotal = 0;
                double power;

                while (data.keepThreadAlive.get()) {
                    if (data.isDirty.compareAndSet(true, false)) {
                        curTarget = data.getTargetPosition();
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

        /**
         * Politely asks the thread to stop.  It may just ignore the request
         */
        public void askToStop() {
            data.keepThreadAlive.set(false);
        }
    }
    private final ArmPositionUpdaterThread armPositionUpdaterThread;

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
        public final AtomicBoolean keepThreadAlive = new AtomicBoolean(true);
        public final AtomicBoolean startUpdateLoop = new AtomicBoolean(false);
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
     * Gets the state of the arm motor's PID loop thread
     * @return The thread's state
     */
    public Thread.State getArmThreadState() {
        return armPositionUpdaterThread.getState();
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
        return ((double)armMotor.requireDevice().getCurrentPosition() * ONE_REVOLUTION_OUR_ANGLE_UNIT / ONE_REVOLUTION_ENCODER_TICKS);
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

    @Override
    public void cleanupModule() {
        armPositionUpdaterThread.askToStop(); // I have no clue if this actually works
        ElapsedTime t = new ElapsedTime();
        while (t.seconds() < 1.5) {
            parent.telemetry.addLine("[Arm] thread is still alive");
            parent.telemetry.update();
            switch (armPositionUpdaterThread.getState()) {
                case BLOCKED:
                case WAITING:
                case TIMED_WAITING:
                    t.reset(); // thread is waiting for something
                    break;
                case TERMINATED:
                    parent.telemetry.addLine("[Arm] Thread stopped");
                    parent.telemetry.update();
                    parent.telemetry.speak("The arm position updater thread has exited gracefully.", "en", "us");
                    return; // thread is done
                default:
                    break; // thread is doing something
            }
        }
        armPositionUpdaterThread.interrupt(); // just kill it if it takes this long

        parent.telemetry.addLine("[Arm] Thread was interrupted");
        parent.telemetry.update();
        parent.telemetry.speak("The arm position updater thread has taken too long to exit, and has been forcefully interrupted.  Be careful when starting the next operating mode.", "en", "us");
    }

    @Override
    public void log() {

    }
}
