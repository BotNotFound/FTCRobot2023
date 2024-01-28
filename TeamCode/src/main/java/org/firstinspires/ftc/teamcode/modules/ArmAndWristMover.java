package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BiPredicate;
import java.util.function.IntPredicate;

final class ArmAndWristMover {
    private final ConditionalHardwareDevice<DcMotor> armMotor;
    private final ConditionalHardwareDevice<Servo> wristServo;

    private final double armEpsilon;
    private final double wristEpsilon;

    private final IntPredicate wristDangerChecker;
    private final double safeWristPosition;
    private final BiPredicate<Integer, Double> pixelSafetyChecker;
    private final MotorPowerCalculator armPowerCalculator;

    private final HardwareInterface hardwareInterface;
    private final AtomicReference<RotationCommand> curCmd;

    public ArmAndWristMover(
            ConditionalHardwareDevice<DcMotor> armMotor,
            ConditionalHardwareDevice<Servo> wristServo,
            int armEpsilon,
            double wristEpsilon,
            IntPredicate wristDangerChecker,
            double safeWristPosition,
            BiPredicate<Integer, Double> pixelSafetyChecker,
            MotorPowerCalculator armPowerCalculator
    ) {
        this.armMotor = armMotor;
        this.wristServo = wristServo;
        this.armEpsilon = armEpsilon;
        this.wristEpsilon = wristEpsilon;
        this.wristDangerChecker = wristDangerChecker;
        this.safeWristPosition = safeWristPosition;
        this.pixelSafetyChecker = pixelSafetyChecker;
        this.armPowerCalculator = armPowerCalculator;
        this.hardwareInterface = new HardwareInterface();
        this.curCmd = new AtomicReference<>(
                new RotationCommand(hardwareInterface.getArmPosition(), hardwareInterface.getWristPosition(), WristRotationMode.DO_NOT_ROTATE)
        );
    }

    private enum WristRotationMode {ASAP, WITHOUT_DROPPING_PIXELS, COMPACT_WHILE_UNSAFE, DO_NOT_ROTATE}

    private final class HardwareInterface {
        private double prevArmPower;
        private double prevWristPosition;

        public synchronized int getArmPosition() {
            return armMotor.isAvailable() ? armMotor.requireDevice().getCurrentPosition() : curCmd.get().getArmTargetPosition();
        }

        public synchronized double getWristPosition() {
            return wristServo.isAvailable() ? wristServo.requireDevice().getPosition() : curCmd.get().getWristTargetPosition();
        }

        public synchronized boolean isArmWithinRangeOf(int expectedPosition) {
            return Math.abs(getArmPosition() - expectedPosition) < armEpsilon;
        }

        public synchronized boolean isWristWithinRangeOf(double expectedPosition) {
            return Math.abs(hardwareInterface.getWristPosition() - expectedPosition) < wristEpsilon;
        }

        public synchronized void setArmPower(double power) {
            if (power == prevArmPower) {
                return;
            }
            prevArmPower = power;
            armMotor.runIfAvailable(arm -> arm.setPower(power));
        }

        public synchronized void setWristPosition(double position) {
            if (position == prevWristPosition) {
                return;
            }
            prevWristPosition = position;
            wristServo.runIfAvailable(wrist -> wrist.setPosition(position));
        }
    }

    private final class RotationCommand {
        private final int armTargetPosition;
        private final double wristTargetPosition;
        private final WristRotationMode wristRotationMode;

        private boolean hasSafetyTriggered;

        public RotationCommand(int armTargetPosition, double wristTargetPosition, WristRotationMode wristRotationMode) {
            this.armTargetPosition = armTargetPosition;
            this.wristTargetPosition = wristTargetPosition;
            this.wristRotationMode = wristRotationMode;
            this.hasSafetyTriggered = false;
        }

        public int getArmTargetPosition() {
            return armTargetPosition;
        }

        public double getWristTargetPosition() {
            return wristTargetPosition;
        }

        public WristRotationMode getWristRotationMode() {
            return wristRotationMode;
        }

        public boolean isArmMovementCompleted() {
            return hardwareInterface.isArmWithinRangeOf(armTargetPosition);
        }

        public boolean isWristMovementCompleted() {
            return wristRotationMode == WristRotationMode.DO_NOT_ROTATE || hardwareInterface.isWristWithinRangeOf(wristTargetPosition);
        }

        public boolean hasMovementCompleted() {
            return isWristMovementCompleted() && isArmMovementCompleted();
        }

        private boolean hasSafetyTriggered() {
            if (!hasSafetyTriggered) {
                hasSafetyTriggered = wristDangerChecker.test(hardwareInterface.getArmPosition());
            }
            return hasSafetyTriggered;
        }

        public boolean isWristUnsafe() {
            return !hasSafetyTriggered() || wristDangerChecker.test(hardwareInterface.getArmPosition());
        }

        public boolean isCommandActive() {
            return curCmd.get() == this;
        }

        public void updateWristMode(WristRotationMode newMode) {
            curCmd.compareAndSet(this, new RotationCommand(this.armTargetPosition, this.wristTargetPosition, newMode));
        }

        @NonNull
        @Override
        public String toString() {
            return "{ Arm: " + armTargetPosition
//                    + "(Done: " + isArmMovementCompleted() + "); "
                    + " Wrist: " + wristTargetPosition
                    + " Wmode: " + wristRotationMode + " }";
//                    + "(Done: " + isWristMovementCompleted() + ") }";
        }
    }

    private RotationCommand makeRotationCommand(int armTargetPosition, double wristTargetPosition) {
        if (wristTargetPosition < 0) { wristTargetPosition = 0; }
        else if (wristTargetPosition > 1) { wristTargetPosition = 1; }

        WristRotationMode wristRotationMode = WristRotationMode.ASAP;

        if (wristDangerChecker.test(armTargetPosition)) {
            wristRotationMode = WristRotationMode.COMPACT_WHILE_UNSAFE;
        }
        else if (!pixelSafetyChecker.test(hardwareInterface.getArmPosition(), wristTargetPosition)) {
            wristRotationMode = WristRotationMode.WITHOUT_DROPPING_PIXELS;
        }

        return new RotationCommand(armTargetPosition, wristTargetPosition, wristRotationMode);
    }

    public void moveArmAndWristAsync(int armTargetPosition, double wristTargetPosition) {
        curCmd.set(makeRotationCommand(armTargetPosition, wristTargetPosition));
    }

    public void setArmTargetPosition(int armTargetPosition) {
        curCmd.getAndUpdate(cmd -> makeRotationCommand(armTargetPosition, cmd.getWristTargetPosition()));
    }

    public void setWristTargetPosition(double wristTargetPosition) {
        curCmd.getAndUpdate(cmd -> makeRotationCommand(cmd.getArmTargetPosition(), wristTargetPosition));
    }

    public void moveArmAndWrist(int armTargetPosition, double wristTargetPosition) {
        final RotationCommand cmd = makeRotationCommand(armTargetPosition, wristTargetPosition);
        curCmd.set(cmd);
        while (!cmd.hasMovementCompleted() && cmd.isCommandActive()) {
            cycleStateMachine();
        }
    }

    public int getArmPosition() {
        return hardwareInterface.getArmPosition();
    }

    public double getWristPosition() {
        return hardwareInterface.getWristPosition();
    }

    public int getArmTargetPosition() {
        return curCmd.get().getArmTargetPosition();
    }

    public double getWristTargetPosition() {
        return curCmd.get().getWristTargetPosition();
    }

    public void cycleStateMachine() {
        final RotationCommand cmd = curCmd.get();

        if (!cmd.isWristMovementCompleted()) {
            switch (cmd.getWristRotationMode()) {
                case DO_NOT_ROTATE:
                    break;
                case ASAP:
                    hardwareInterface.setWristPosition(cmd.getWristTargetPosition());
                    break;
                case COMPACT_WHILE_UNSAFE:
                    if (cmd.isArmMovementCompleted()) {
                        cmd.updateWristMode(WristRotationMode.ASAP);
                    }
                    else if (cmd.isWristUnsafe()) {
                        hardwareInterface.setWristPosition(safeWristPosition);
                        if (!hardwareInterface.isWristWithinRangeOf(safeWristPosition)) {
                            return;
                        }
                    }
                    else {
                        cmd.updateWristMode(WristRotationMode.WITHOUT_DROPPING_PIXELS);
                    }
                    break;
                case WITHOUT_DROPPING_PIXELS:
                    if (cmd.isArmMovementCompleted()) {
                        cmd.updateWristMode(WristRotationMode.ASAP);
                    }
                    else {
                        hardwareInterface.setWristPosition(safeWristPosition - (getArmTargetPosition() / Arm.ONE_REVOLUTION_ENCODER_TICKS));
                    }
                    break;
            }
        }

        if (!cmd.isArmMovementCompleted()) {
            hardwareInterface.setArmPower(armPowerCalculator.calculateMotorPower(hardwareInterface.getArmPosition(), cmd.getArmTargetPosition()));
        } else {
            hardwareInterface.setArmPower(0.0);
        }
    }

    public String getStatusString() {
        return curCmd.get().toString();
    }
}
