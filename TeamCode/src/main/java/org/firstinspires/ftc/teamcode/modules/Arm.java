package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.MathUtils;
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
@Config
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

    public static final double ARM_ANGLE_OFFSET = ANGLE_UNIT.fromDegrees(-29.208);

    /**
     * One full rotation in the unit specified by {@link #ANGLE_UNIT}
     */
    public static final double ONE_REVOLUTION_OUR_ANGLE_UNIT = ANGLE_UNIT.getUnnormalized().fromDegrees(360.0);

    /**
     * Rotate flap to open position
     */
    public static double FLAP_OPEN_RIGHT = 0.4;

    public static double FLAP_OPEN_LEFT = 0.6;

    /**
     * Rotate flap to closed position
     */
    public static double FLAP_CLOSED = 0.53;

    public static final class ArmPresets extends Presets {
        /**
         * Rotates the arm to the position it was in at the start of execution.  This should be parallel to the ground,
         *  with the end of the arm closest to the active intake.
         */
        public static final double IDLE = 25.0;

        /**
         * Rotates the arm so that the robot can collect pixels
         */
        public static final double READY_TO_INTAKE = 0.0;

        /**
         * Rotates the arm so that the robot can deposit pixels on the floor behind the active intake
         */
        public static final double DEPOSIT_ON_FLOOR = 200.0;

        /**
         * Rotates the arm so that the robot can deposit pixels on the backdrop behind the active intake
         */
        public static final double DEPOSIT_ON_BACKDROP = 155.0;
    }

    public static final class WristPresets extends Presets { // TODO these presets are untested
        /**
         * Rotates the wrist to the position it was in at the start of execution.
         * This should be parallel to the ground.
         */
        public static final double IDLE = 1;
        /**
         * Rotates the wrist so that the robot can collect pixels
         */
        public static final double READY_TO_INTAKE = 0.565;

        /**
         * Rotates the wrist so that the robot can deposit pixels on the floor behind the active intake
         */
        public static final double DEPOSIT_ON_FLOOR = 0.5;

        /**
         * Rotates the wrist so that the robot can deposit pixels on the backdrop behind the active intake
         */
        public static final double DEPOSIT_ON_BACKDROP = 0.45;
    }

    //private boolean isFlapOpen;
    private FlapState currentFlapState;

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

        setFlapState(FlapState.CLOSED);

        exitSetup();

    }

    /**
     * A thread that keeps the arm motor at the target position
     */
    @Config("Arm (Position Updater Thread)")
    private static class ArmPositionUpdaterThread extends ModuleThread<Arm> {
        public static final String THREAD_NAME = "Arm Position Updater";

        public static double kP = 0.000945;
        public static double kI = 0.001;
        public static double kD = 0;

        public static double INTEGRAL_MAX_POWER = 0.05;

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
                boolean adjustWristPosition = false;
                int currentPosition;

                while (host.getState().isRunning()) {
                    if (host.armData.isDirty.compareAndSet(true, false)) {
                        curTarget = host.armData.getTargetPosition();
                        adjustWristPosition = host.armData.getAdjustWristPosition();

                        // if we never made it to the target (i.e. we're tuning the PID controller and kI has been 0 for
                        //  a while), we don't want a potentially massive error total to roll over to our new position
                        //  and potentially be detrimental/dangerous
                        errorTotal = 0;
                    }

                    currentPosition = arm.getCurrentPosition();
                    if (adjustWristPosition) {
                        //host.rotateWristTo(host.getWristRotation() - host.getArmRotation());
                    }

                    error = currentPosition - curTarget;
                    errorChange = error - prevError;
                    errorTotal += error;
                    errorTotal = (int)(Math.min(Math.abs(errorTotal), INTEGRAL_MAX_POWER / kI) * Math.signum(errorTotal)); // integral sum limit (errorTotal * kI <= INTEGRAL_MAX_POWER)

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
        private boolean adjustWristPosition = false;
        private final Object dataMonitor = new Object(); // I'm pretty sure this is how threads work
        public int getTargetPosition() {
            synchronized (dataMonitor) {
                return targetPosition;
            }
        }
        public void setTargetPosition(int newTarget) {
            if (targetPosition == newTarget) {
                return; // nothing to update
            }

            synchronized (dataMonitor) {
                targetPosition = newTarget;
                isDirty.set(true);
            }
        }
        public boolean getAdjustWristPosition() {
            synchronized (dataMonitor) {
                return adjustWristPosition;
            }
        }
        public void setAdjustWristPosition(boolean newValue) {
            synchronized (dataMonitor) {
                adjustWristPosition = newValue;
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
        final double normalizedAngle = normalizeAngleOurWay(rotation, angleUnit);

        // These presets are the most we will ever need to rotate the arm, so we can use them to prevent unwanted rotation
        if (normalizedAngle > ArmPresets.DEPOSIT_ON_FLOOR || normalizedAngle < ArmPresets.READY_TO_INTAKE ) {
            return; // don't rotate the arm into the floor
        }

        armData.setTargetPosition((int)Math.round(
                normalizedAngle
                        * ONE_REVOLUTION_ENCODER_TICKS // multiply before dividing to retain maximum precision
                        / ONE_REVOLUTION_OUR_ANGLE_UNIT
        ));

        armData.setAdjustWristPosition(preserveWristRotation);
    }

    /**
     * Normalizes the given angle so that it is within one positive rotation (0-360 degrees, or 0-2pi radians)
     * @param angle The angle to normalize
     * @param unitUsed The unit of the given angle
     * @return The normalized angle
     */
    public static double normalizeAngleOurWay(double angle, AngleUnit unitUsed) {
        angle = ANGLE_UNIT.getUnnormalized().fromUnit(unitUsed.getUnnormalized(), angle);

        angle = angle % ONE_REVOLUTION_OUR_ANGLE_UNIT; // normalize from 1 full negative revolution up to 1 full positive revolution

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
     * Rotates the arm to the specified rotation
     * @param rotation The target rotation, in {@link #ANGLE_UNIT}s
     * @param preserveWristRotation should the wrist rotate with the arm so that it is facing the same direction at the end of rotation?
     */
    public void rotateArmTo(double rotation, boolean preserveWristRotation) {
        rotateArmTo(rotation, ANGLE_UNIT, preserveWristRotation);
    }

    /**
     * Rotates the arm to the specified rotation, WITHOUT preserving wrist rotation.
     * @param rotation The target rotation, in {@link #ANGLE_UNIT}s
     */
    public void rotateArmTo(double rotation) {
        rotateArmTo(rotation, false);
    }

    /**
     * Rotates the wrist to the specified rotation
     * @param rotation The target rotation
     * @param angleUnit The unit of rotation used
     */
    public void rotateWristTo(double rotation, AngleUnit angleUnit) {
        wristServo.runIfAvailable((Servo wrist) -> {
            //final double convertedRotation = ANGLE_UNIT.fromUnit(angleUnit, rotation); // convert angle to our unit
            wrist.setPosition(MathUtils.clamp(rotation, 0.35, 0.85));//convertedRotation * ONE_REVOLUTION_ENCODER_TICKS / ONE_REVOLUTION_OUR_ANGLE_UNIT);
        });
    }

    /**
     * Rotates the wrist to the specified rotation
     * @param rotation The target rotation, in {@link #ANGLE_UNIT}s
     */
    public void rotateWristTo(double rotation) {
        rotateWristTo(rotation, ANGLE_UNIT);
    }

    /**
     * Gets the rotation of the arm
     * @return The arm's rotation in the unit specified by {@link #ANGLE_UNIT}
     */
    public double getArmRotation() {
        return ((double)getArmMotorPosition() * ONE_REVOLUTION_OUR_ANGLE_UNIT / ONE_REVOLUTION_ENCODER_TICKS) - ARM_ANGLE_OFFSET;
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
     * Gets the rotation of the wrist
     * @param angleUnit The unit of rotation to use
     * @return The wrist's rotation in the unit specified
     */
    public double getWristRotation(AngleUnit angleUnit) {
        return angleUnit.fromUnit(ANGLE_UNIT, getWristRotation());
    }

    public void cycleFlap() {
        flapServo.runIfAvailable(flap -> {
            switch (currentFlapState) {
                case OPEN_RIGHT:
                    setFlapState(FlapState.OPEN_LEFT);
                    break;
                case OPEN_LEFT:
                    setFlapState(FlapState.CLOSED);
                    break;
                case CLOSED:
                    setFlapState(FlapState.OPEN_RIGHT);
                    break;
            }
        });
    }

    public static final long FLAP_CYCLE_WAIT_TIME = 200;

    public void fullCycleFlap() {
        try {
            setFlapState(FlapState.CLOSED);
            cycleFlap();
            Thread.sleep(FLAP_CYCLE_WAIT_TIME);
            cycleFlap();
            Thread.sleep(FLAP_CYCLE_WAIT_TIME);
            cycleFlap();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Sets the currentFlapState, as well moving the flap to that state
     * @param flapState The FLAP_STATE to set the flap to
     */
    private void setFlapState(FlapState flapState) {
        currentFlapState = flapState;
        flapServo.runIfAvailable(flap -> flap.setPosition(flapState.targetFlapPosition));
    }

    public void closeFlap() {
        setFlapState(FlapState.CLOSED);
    }

    public boolean isFlapOpenRight() {
        flapServo.requireDevice();
        return currentFlapState == FlapState.OPEN_RIGHT;
    }

    public boolean isFlapOpenLeft() {
        flapServo.requireDevice();
        return  currentFlapState == FlapState.OPEN_LEFT;
    }

    /**
     * Opens the flap, if the flap is not already open
     */
    /*public void openFlap() {
        flapServo.runIfAvailable(flap -> {
            if (isFlapOpen) {
                return;
            }
            flap.setPosition(FLAP_OPEN);
            isFlapOpen = true;
        });
    }*/

    /**
     * Closes the flap, if the flap is not already closed
     */
    /*public void closeFlap() {
        flapServo.runIfAvailable(flap -> {
            if (!isFlapOpen) {
                return;
            }
            flap.setPosition(FLAP_CLOSED);
            isFlapOpen = false;
        });
    }*/

    /**
     * If the flap is open, close it.  Otherwise, open the flap
     */
    /*public void toggleFlap() {
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
    }*/

    @Override
    protected void registerModuleThreads() {
        registerAsyncOperation(new ArmPositionUpdaterThread(this));
    }

    @Override
    public void log() {
        getTelemetry().addData("[Arm] module state", getState());
        armMotor.runIfAvailable(arm -> getTelemetry().addData( "[Arm] (arm motor) current rotation",
                Math.rint(getArmRotation(AngleUnit.DEGREES) * 100) / 100 ));
        wristServo.runIfAvailable(wrist -> getTelemetry().addData( "[Arm] (wrist servo) current rotation",
                Math.rint(getWristRotation(AngleUnit.DEGREES) * 100) / 100 ));
        flapServo.runIfAvailable(flap -> getTelemetry().addData("[Arm] is the flap open", !(isFlapOpenLeft() && isFlapOpenRight())));
    }

    private enum FlapState {
        OPEN_RIGHT(FLAP_OPEN_RIGHT),
        OPEN_LEFT(FLAP_OPEN_LEFT),
        CLOSED(FLAP_CLOSED);

        public final double targetFlapPosition;

        FlapState(double targetFlapPosition) {
            this.targetFlapPosition = targetFlapPosition;
        }
    }
}