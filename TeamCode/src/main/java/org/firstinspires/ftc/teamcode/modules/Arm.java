package org.firstinspires.ftc.teamcode.modules;

import android.util.Range;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.GearRatio;
import org.firstinspires.ftc.teamcode.modules.core.Module;

import java.util.function.DoubleUnaryOperator;

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

    public static final double ARM_ANGLE_OFFSET = ANGLE_UNIT.fromDegrees(-29.208);

    /**
     * One full rotation in the unit specified by {@link #ANGLE_UNIT}
     */
    public static final double ONE_REVOLUTION_OUR_ANGLE_UNIT = ANGLE_UNIT.getUnnormalized().fromDegrees(360.0);

    /**
     * Rotate flap to open position with the right pixel exposed
     */
    public static final double FLAP_OPEN_RIGHT = 0.4;

    /**
     * Rotate flap to open position with the left pixel exposed
     */
    public static final double FLAP_OPEN_LEFT = 0.6;

    /**
     * Rotate flap to closed position
     */
    public static final double FLAP_CLOSED = 0.53;

    private static final Range<Double> WRIST_VALID_POSITION_RANGE = new Range<>(0.35, 0.85);
    public static double kP = 0.000945;
    public static double kI = 0.001;
    public static double kD = 0;
    public static double INTEGRAL_MAX_POWER = 0.05;

    @Config
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

    @Config
    public static final class WristPresets extends Presets {
        /**
         * Rotates the wrist to the position it was in at the start of execution.
         * This should be parallel to the ground.
         */
        public static final double IDLE = 0.75;
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
     * The state machine that moves the arm and wrist
     */
    private final ArmAndWristMover armAndWristMover;

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
        armAndWristMover = new ArmAndWristMover(
                armMotor,
                wristServo,
                100,
                0.001,
                Arm::isWristInDanger,
                0.75,
                Arm::willPixelsFallOut,
                new PIDAlgorithm(
                        kP,
                        kI,
                        kD,
                        DoubleUnaryOperator.identity(),
                        PIDAlgorithm.limitIntegralTermTo((long)Math.ceil(INTEGRAL_MAX_POWER))
                )
        );

        setFlapState(FlapState.CLOSED);
    }

    public void doArmUpdateLoop() {
        armAndWristMover.cycleStateMachine();
    }

    private static boolean isWristInDanger(int armPosition) {
        return armPosition < ONE_REVOLUTION_ENCODER_TICKS * 2 / 3;
    }

    private static boolean willPixelsFallOut(int armPosition, double wristPosition) {
        double armRotationDegrees = armPosition * 360.0 / ONE_REVOLUTION_ENCODER_TICKS;
        double wristRotationDegrees = wristPosition * 360.0;
        double pixelRotationDegrees = armRotationDegrees + wristRotationDegrees;
        return Math.abs(pixelRotationDegrees) > 75.0;
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
        return armAndWristMover.getArmTargetPosition();
    }

    private int calculateArmPosition(double angle) {
        return (int)Math.round(
                (angle + ARM_ANGLE_OFFSET)
                        * ONE_REVOLUTION_ENCODER_TICKS // multiply before dividing to retain maximum precision
                        / ONE_REVOLUTION_OUR_ANGLE_UNIT
        );
    }

    private boolean isArmRotationNotAllowed(double angle) {
        return angle >= ArmPresets.DEPOSIT_ON_FLOOR || angle <= ArmPresets.READY_TO_INTAKE;
    }

    /**
     * Rotates the arm to the specified rotation
     * @param rotation The target rotation
     * @param angleUnit The unit of rotation used
     * @param preserveWristRotation should the wrist rotate with the arm so that it is facing the same direction at the end of rotation?
     */
    public void rotateArmToAsync(double rotation, AngleUnit angleUnit, boolean preserveWristRotation) {
        final double normalizedAngle = normalizeAngleOurWay(rotation - ARM_ANGLE_OFFSET, angleUnit);

        // These presets are the most we will ever need to rotate the arm, so we can use them to prevent unwanted rotation
        if (isArmRotationNotAllowed(normalizedAngle)) {
            return; // don't rotate the arm into the floor
        }

        final int armTargetPosition = calculateArmPosition(normalizedAngle);

        if (preserveWristRotation) {
            final double targetWristPosition = armAndWristMover.getWristTargetPosition() + (normalizedAngle / ONE_REVOLUTION_OUR_ANGLE_UNIT);
            armAndWristMover.moveArmAndWristAsync(armTargetPosition, targetWristPosition);
        }
        else {
            armAndWristMover.setArmTargetPosition(armTargetPosition);
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
     * @param rotation The target rotation
     * @param angleUnit The unit of rotation used
     */
    public void rotateArmToAsync(double rotation, AngleUnit angleUnit) {
        rotateArmToAsync(rotation, angleUnit, false);
    }

    /**
     * Rotates the arm to the specified rotation
     * @param rotation The target rotation, in {@link #ANGLE_UNIT}s
     * @param preserveWristRotation should the wrist rotate with the arm so that it is facing the same direction at the end of rotation?
     */
    public void rotateArmToAsync(double rotation, boolean preserveWristRotation) {
        rotateArmToAsync(rotation, ANGLE_UNIT, preserveWristRotation);
    }

    /**
     * Rotates the arm to the specified rotation, WITHOUT preserving wrist rotation.
     * @param rotation The target rotation, in {@link #ANGLE_UNIT}s
     */
    public void rotateArmToAsync(double rotation) {
        rotateArmToAsync(rotation, false);
    }

    /**
     * Rotates the wrist to the specified position
     * @param position The target position.  This value must be between 0.0 and 1.0 (inclusive)
     */
    public void rotateWristToAsync(double position) {
        final double clampedPosition = WRIST_VALID_POSITION_RANGE.clamp(position);
        armAndWristMover.setWristTargetPosition(clampedPosition);
    }

    public void rotateArmAndWrist(double armRotation, double wristPosition) {
        if (isArmRotationNotAllowed(armRotation)) {
            return;
        }

        final int armPosition = calculateArmPosition(armRotation);

        armAndWristMover.moveArmAndWrist(armPosition, wristPosition);
    }

    /**
     * Gets the rotation of the arm
     * @return The arm's rotation in the unit specified by {@link #ANGLE_UNIT}
     */
    public double getArmRotation() {
        return ((double)getArmMotorPosition() * ONE_REVOLUTION_OUR_ANGLE_UNIT / ONE_REVOLUTION_ENCODER_TICKS) + ARM_ANGLE_OFFSET;
    }

    /**
     * Gets the rotation of the arm
     * @param angleUnit The unit of rotation to use
     * @return The arm's rotation in the unit specified
     */
    public double getArmRotation(AngleUnit angleUnit) {
        return angleUnit.fromUnit(ANGLE_UNIT, getArmRotation()) - ARM_ANGLE_OFFSET;
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
     * Cycles the flap's state from OPEN_RIGHT -> OPEN_LEFT -> CLOSE -> OPEN_RIGHT and etc.
     */
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

    public  static final long FLAP_CYCLE_WAIT_TIME = 200;

    /**
     * Cycles through all flap states with enough time for pixels to fall out
     */
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
        return currentFlapState == FlapState.OPEN_LEFT;
    }

    @Override
    public void log() {
        getTelemetry().addData("[Arm] Movement status", armAndWristMover.getStatusString());
        armMotor.runIfAvailable(arm -> getTelemetry().addData( "[Arm] (arm motor) current rotation",
                Math.rint(getArmRotation(AngleUnit.DEGREES) * 100) / 100 ));
        wristServo.runIfAvailable(wrist -> getTelemetry().addData( "[Arm] (wrist servo) current rotation",
                Math.rint(getWristRotation(AngleUnit.DEGREES) * 100) / 100 ));
        flapServo.runIfAvailable(flap -> getTelemetry().addData("[Arm] is the flap closed", !(isFlapOpenLeft() && isFlapOpenRight())));
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

    /**
     * Used for logging from modules
     */
    @Override
    public Telemetry getTelemetry() {
        return super.getTelemetry();
    }

    /**
     * Ran by parent OpMode in its stop() method
     * Cleans up items like background threads
     */
    @Override
    public void cleanupModule() {

    }
}
