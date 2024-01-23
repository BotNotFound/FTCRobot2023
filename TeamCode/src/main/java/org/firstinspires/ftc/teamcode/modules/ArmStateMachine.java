package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BiFunction;
import java.util.function.BiPredicate;
import java.util.function.IntPredicate;

final class ArmStateMachine {
    private final ConditionalHardwareDevice<DcMotor> armMotor;
    private final ConditionalHardwareDevice<Servo> wristServo;

    private final double wristEpsilon;

    private final IntPredicate wristDangerChecker;
    private final BiPredicate<Integer, Double> pixelSafetyChecker;
    private final BiFunction<Integer, Integer, Double> armPowerCalculator;

    private final HardwareInterface hardwareInterface;
    private final AtomicReference<RotationCommand> curCmd;

    public ArmStateMachine(
            ConditionalHardwareDevice<DcMotor> armMotor,
            ConditionalHardwareDevice<Servo> wristServo,
            double wristEpsilon,
            IntPredicate wristDangerChecker,
            BiPredicate<Integer, Double> pixelSafetyChecker,
            BiFunction<Integer, Integer, Double> armPowerCalculator
    ) {
        this.armMotor = armMotor;
        this.wristServo = wristServo;
        this.wristEpsilon = wristEpsilon;
        this.wristDangerChecker = wristDangerChecker;
        this.pixelSafetyChecker = pixelSafetyChecker;
        this.armPowerCalculator = armPowerCalculator;
        this.hardwareInterface = new HardwareInterface();
        this.curCmd = new AtomicReference<>(
                new RotationCommand(hardwareInterface.getArmPosition(), hardwareInterface.getWristPosition(), WristRotationMode.DO_NOT_ROTATE)
        );
    }

    private enum WristRotationMode { ASAP, WITHOUT_DROPPING_PIXELS, FINISH_BEFORE_ARM_ROTATION, DO_NOT_ROTATE };

    private final class HardwareInterface {
        private double prevArmPower;
        private double prevWristPosition;

        public synchronized int getArmPosition() {
            return armMotor.isAvailable() ? armMotor.requireDevice().getCurrentPosition() : 0;
        }

        public synchronized double getWristPosition() {
            return wristServo.isAvailable() ? wristServo.requireDevice().getPosition() : 0.0;
        }

        public synchronized void setArmPower(double power) {
            if (power == prevArmPower) { return; }
            armMotor.runIfAvailable(arm -> arm.setPower(power));
        }

        public synchronized void setWristPosition(double position) {
            if (position == prevWristPosition) { return; }
            wristServo.runIfAvailable(wrist -> wrist.setPosition(position));
        }
    }

    private static final class RotationCommand {
        private final int armTargetPosition;
        private final double wristTargetPosition;
        private final WristRotationMode wristRotationMode;

        public RotationCommand(int armTargetPosition, double wristTargetPosition, WristRotationMode wristRotationMode) {
            this.armTargetPosition = armTargetPosition;
            this.wristTargetPosition = wristTargetPosition;
            this.wristRotationMode = wristRotationMode;
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
    }

    public void setArmTargetPosition(int armTargetPosition) {
        curCmd.set(new RotationCommand(armTargetPosition, 0.0, WristRotationMode.DO_NOT_ROTATE));
    }

    public void setWristTargetPosition(double wristTargetPosition) {
        curCmd.set(new RotationCommand(hardwareInterface.getArmPosition(), wristTargetPosition, WristRotationMode.ASAP));
    }

    public void moveArmAndWrist(int armTargetPosition, double wristTargetPosition) {
        WristRotationMode wristRotationMode = WristRotationMode.ASAP;
        if (!pixelSafetyChecker.test(hardwareInterface.getArmPosition(), wristTargetPosition)) {
            wristRotationMode = WristRotationMode.WITHOUT_DROPPING_PIXELS;
        } else if (wristDangerChecker.test(armTargetPosition)) {
            wristRotationMode = WristRotationMode.FINISH_BEFORE_ARM_ROTATION;
        }
        curCmd.set(new RotationCommand(armTargetPosition, wristTargetPosition, wristRotationMode));
    }

    public void cycleStateMachine() {
        final RotationCommand cmd = curCmd.get();
        if (Math.abs(cmd.getWristTargetPosition() - hardwareInterface.getWristPosition()) > wristEpsilon) {
            switch (cmd.wristRotationMode) {
                case DO_NOT_ROTATE:
                    break;
                case ASAP:
                    hardwareInterface.setWristPosition(cmd.getWristTargetPosition());
                    break;
                case FINISH_BEFORE_ARM_ROTATION:
                    hardwareInterface.setWristPosition(cmd.getWristTargetPosition());
                    return;
                case WITHOUT_DROPPING_PIXELS:
                    if (pixelSafetyChecker.test(hardwareInterface.getArmPosition(), cmd.getWristTargetPosition())) {
                        hardwareInterface.setWristPosition(cmd.getWristTargetPosition());
                    }
                    break;
            }
        }

        if (hardwareInterface.getArmPosition() != cmd.getArmTargetPosition()) {
            hardwareInterface.setArmPower(armPowerCalculator.apply(hardwareInterface.getArmPosition(), cmd.getArmTargetPosition()));
        }
        else {
            hardwareInterface.setArmPower(0.0);
        }
    }
}
