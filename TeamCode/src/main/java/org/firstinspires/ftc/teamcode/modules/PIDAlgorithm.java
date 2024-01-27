package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleUnaryOperator;
import java.util.function.LongUnaryOperator;

public class PIDAlgorithm implements MotorPowerCalculator {
    private long totalError;
    private int prevError;
    private final ElapsedTime elapsedTime;
    private int targetPosition;
    private final double proportionalCoefficient;
    private final double integralCoefficient;
    private final double derivativeCoefficient;
    private final DoubleUnaryOperator derivativeTermModifier;
    private final LongUnaryOperator integralTermModifier;

    public PIDAlgorithm(
            double proportionalCoefficient,
            double integralCoefficient,
            double derivativeCoefficient,
            DoubleUnaryOperator derivativeTermModifier,
            LongUnaryOperator integralTermModifier
    ) {
        this.proportionalCoefficient = proportionalCoefficient;
        this.integralCoefficient = integralCoefficient;
        this.derivativeCoefficient = derivativeCoefficient;
        this.elapsedTime = new ElapsedTime();
        this.derivativeTermModifier = derivativeTermModifier;
        this.integralTermModifier = integralTermModifier;
        setTargetPosition(0);
    }

    private static final long SIGN_BIT = ~(Long.MIN_VALUE & Long.MAX_VALUE);
    private static long signOf(long l) {
        if (l == 0L) {
            return 0L;
        }

        final long justSignBit = l & SIGN_BIT;
        if (justSignBit == 0) {
            return 1L; // value is not zero, so it must be positive
        }

        return -1L; // value is not positive or zero; it must be negative
    }

    public static LongUnaryOperator limitIntegralTermTo(long absLimit) {
        return i -> Math.min(Math.abs(i), absLimit) * signOf(i);
    }

    protected int calculateError(int currentPosition, int targetPosition) {
        return currentPosition - targetPosition;
    }
    protected double calculateChangeInError(int currentError, int previousError, double deltaTime) {
        final double actualChange = (currentError - previousError) / deltaTime;
        return derivativeTermModifier.applyAsDouble(actualChange);
    }
    protected long calculateTotalError(int currentError, long previousTotalError) {
        final long actualTotal = currentError + previousTotalError;
        return integralTermModifier.applyAsLong(actualTotal);
    }

    private void setTargetPosition(int newTarget) {
        if (targetPosition == newTarget) { return; }

        this.elapsedTime.reset();
        this.prevError = 0;
        this.totalError = 0;
        this.targetPosition = newTarget;
    }

    @Override
    public double calculateMotorPower(int currentPosition, int targetPosition) {
        setTargetPosition(targetPosition);

        final double deltaTime = elapsedTime.milliseconds();
        elapsedTime.reset();

        final int error = calculateError(currentPosition, targetPosition);
        final double errorChange = calculateChangeInError(error, prevError, deltaTime);
        totalError = calculateTotalError(error, this.totalError);

        prevError = error;

        double calculatedPower = (error * proportionalCoefficient) + (totalError * integralCoefficient) + (errorChange * derivativeCoefficient);
        return MOTOR_POWER_RANGE.clamp(calculatedPower);
    }
}
