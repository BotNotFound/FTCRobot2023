package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.GearRatio;
import org.firstinspires.ftc.teamcode.modules.core.Module;

public final class Arm extends Module {
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

    private static final double ARM_ANGLE_OFFSET = ANGLE_UNIT.fromDegrees(-29.208);

    /**
     * One full rotation in the unit specified by {@link #ANGLE_UNIT}
     */
    public static final double ONE_REVOLUTION_OUR_ANGLE_UNIT = ANGLE_UNIT.getUnnormalized().fromDegrees(360.0);

    /**
     * Rotate flap to open position
     */
    public static final double FLAP_OPEN = 0;

    /**
     * Rotate flap to closed position
     */
    public static final double FLAP_CLOSED = 1;

    public static final class ArmPresets extends Presets {
        /**
         * Rotates the arm to the idle position.  This should be parallel to the ground,
         *  with the end of the arm closest to the active intake.
         */
        public static final double IDLE = 0.0;

        /**
         * Rotates the arm so that the robot can collect pixels
         */
        public static final double READY_TO_INTAKE = -25.0;

        /**
         * Rotates the arm so that the robot can deposit pixels on the floor behind the active intake
         */
        public static final double DEPOSIT_ON_FLOOR = 200.0;

        /**
         * Rotates the arm so that the robot can deposit pixels on the backdrop behind the active intake
         */
        public static final double DEPOSIT_ON_BACKDROP = 115.0;
    }

    public static final class WristPresets extends Presets { // TODO these presets are untested
        /**
         * Rotates the wrist to idle position.
         * This should be parallel to the ground.
         */
        public static final double IDLE = 90.0;
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

        armState = ArmState.createEmptyState();

        // status update
        armMotor.runIfAvailable(
                device -> {
                    device.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    device.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    device.setDirection(DcMotorSimple.Direction.FORWARD);
                    device.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armState = ArmState.fromCurrentPosition(device);
                    getTelemetry().addLine("[Arm] found arm motor of type " + device.getDeviceName() + " on port " + device.getPortNumber());
                },
                () -> getTelemetry().addLine("[Arm] could not find arm motor!")
        );
        wristServo.runIfAvailable(
                device -> {
                    getTelemetry().addLine("[Arm] found wrist servo of type " + device.getDeviceName() + " on port " + device.getPortNumber());
                    device.setPosition(0.5);
                },
                () -> getTelemetry().addLine("[Arm] could not find wrist servo!")
        );
        flapServo.runIfAvailable(
                device -> getTelemetry().addLine("[Arm] found flap servo of type " + device.getDeviceName() + " on port " + device.getPortNumber()),
                () -> getTelemetry().addLine("[Arm] could not find flap servo!")
        );

        isFlapOpen = true;
        closeFlap();
    }

    public static double kP = 0.000945;
    public static double kI = 0.001;
    public static double kD = 0;

    public static double INTEGRAL_MAX_POWER = 0.05;

    public static class ArmState {
        private final int targetPosition;
        private final int prevError;
        private final long totalError;
        private final ElapsedTime elapsedTime;


        private ArmState(int targetPosition, int prevError, long totalError, ElapsedTime timer) {
            this.targetPosition = targetPosition;
            this.prevError = prevError;
            this.totalError = totalError;
            elapsedTime = timer;
            elapsedTime.reset();
        }

        private static ArmState fromCurrentPosition(DcMotor armMotor) {
            return new ArmState(armMotor.getCurrentPosition(), 0, 0, new ElapsedTime());
        }

        private static ArmState createEmptyState() {
            return new ArmState(0, 0, 0, new ElapsedTime());
        }

        private static ArmState genNewRotateCommand(int targetPosition) {
            return new ArmState(targetPosition, 0, 0, new ElapsedTime());
        }
    }

    private ArmState armState;

    private ArmState internalCycleArmPID(ArmState curState) {
        if (!armMotor.isAvailable()) {
            return curState; // cannot move nonexistent arm
        }
        final DcMotor arm = armMotor.requireDevice();

        if (curState == null) {
            return ArmState.fromCurrentPosition(arm); // if null, don't move
        }

        final int currentPosition = arm.getCurrentPosition();
        final long deltaTime = curState.elapsedTime.nanoseconds();

        final int error = currentPosition - curState.targetPosition;
        final double errorChange = (double)(error - curState.prevError) / deltaTime;
        final long errorTotal = curState.totalError + (error / deltaTime);
        final int clampedErrorTotal = (int)(Math.min(Math.abs(errorTotal), INTEGRAL_MAX_POWER / kI) * Math.signum(errorTotal)); // integral sum limit (errorTotal * kI <= INTEGRAL_MAX_POWER)

        final double power = error == 0 ? 0 : (error * kP) + (errorChange * kD) + (clampedErrorTotal * kI);
        arm.setPower(power);
        return new ArmState(curState.targetPosition, error, clampedErrorTotal, curState.elapsedTime);
    }

    public void cycleArmPID() {
        armState = internalCycleArmPID(armState);
    }

    public void beginRotateArmTo(double rotation, AngleUnit angleUnit) {
        if (!armMotor.isAvailable()) {
            return;
        }

        final double normalizedAngle = normalizeAngleOurWay(rotation + angleUnit.fromUnit(ANGLE_UNIT, ARM_ANGLE_OFFSET), angleUnit);

        // These presets are the most we will ever need to rotate the arm, so we can use them to prevent unwanted rotation
        if (normalizedAngle > ArmPresets.DEPOSIT_ON_FLOOR || normalizedAngle < ArmPresets.READY_TO_INTAKE) {
            return; // don't rotate the arm into the floor
        }

        final int targetPosition = (int)Math.round(
                normalizedAngle
                        * ONE_REVOLUTION_ENCODER_TICKS // multiply before dividing to retain maximum precision
                        / ONE_REVOLUTION_OUR_ANGLE_UNIT
        );
        if (targetPosition == armState.targetPosition) {
            return;
        }

        armState = ArmState.genNewRotateCommand(targetPosition);
    }

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
        return armState.targetPosition;
    }

    /**
     * Rotates the arm to the specified rotation
     * @param rotation The target rotation
     * @param angleUnit The unit of rotation used
     */
    public void rotateArmTo(double rotation, AngleUnit angleUnit) {
        if (!armMotor.isAvailable()) {
            return;
        }
        beginRotateArmTo(rotation, angleUnit);

        while (getArmMotorPosition() != armState.targetPosition) {
            cycleArmPID();
        }
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
     * @param rotation The target rotation, in {@link #ANGLE_UNIT}s
     */
    public void rotateArmTo(double rotation) {
        rotateArmTo(rotation, ANGLE_UNIT);
    }

    /**
     * Rotates the wrist to the specified rotation
     * @param rotation The target rotation
     * @param angleUnit The unit of rotation used
     */
    public void rotateWristTo(double rotation, AngleUnit angleUnit) {
        wristServo.runIfAvailable((Servo wrist) -> {
            final double convertedRotation = ANGLE_UNIT.fromUnit(angleUnit, rotation); // convert angle to our unit
            wrist.setPosition(convertedRotation * ONE_REVOLUTION_ENCODER_TICKS / ONE_REVOLUTION_OUR_ANGLE_UNIT);
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
    /**
     * Opens the flap, if the flap is not already open
     */
    public void openFlap() {
        flapServo.runIfAvailable(flap -> {
            if (isFlapOpen) {
                return;
            }
            flap.setPosition(FLAP_OPEN);
            isFlapOpen = true;
        });
    }

    /**
     * Closes the flap, if the flap is not already closed
     */
    public void closeFlap() {
        flapServo.runIfAvailable(flap -> {
            if (!isFlapOpen) {
                return;
            }
            flap.setPosition(FLAP_CLOSED);
            isFlapOpen = false;
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
    public void log() {
        armMotor.runIfAvailable(arm -> getTelemetry().addData( "[Arm] (arm motor) current rotation",
                Math.rint(getArmRotation(AngleUnit.DEGREES) * 100) / 100 ));
        wristServo.runIfAvailable(wrist -> getTelemetry().addData( "[Arm] (wrist servo) current rotation",
                Math.rint(getWristRotation(AngleUnit.DEGREES) * 100) / 100 ));
        flapServo.runIfAvailable(flap -> getTelemetry().addData("[Arm] is the flap open", isFlapOpen()));
    }
}
