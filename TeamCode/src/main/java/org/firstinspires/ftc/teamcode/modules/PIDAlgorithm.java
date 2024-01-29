package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

public class PIDAlgorithm implements MotorPowerCalculator {
    private double totalError;
    private int prevError;
    private final ElapsedTime elapsedTime;
    private int targetPosition;
    private final DoubleSupplier proportionalCoefficientSupplier;
    private final DoubleSupplier integralCoefficientSupplier;
    private final DoubleSupplier derivativeCoefficientSupplier;
    private final DoubleUnaryOperator derivativeTermModifier;
    private final DoubleUnaryOperator integralTermModifier;

    public PIDAlgorithm(
            DoubleSupplier proportionalCoefficientSupplier,
            DoubleSupplier integralCoefficientSupplier,
            DoubleSupplier derivativeCoefficientSupplier,
            DoubleUnaryOperator derivativeTermModifier,
            DoubleUnaryOperator integralTermModifier
    ) {
        this.proportionalCoefficientSupplier = proportionalCoefficientSupplier;
        this.integralCoefficientSupplier = integralCoefficientSupplier;
        this.derivativeCoefficientSupplier = derivativeCoefficientSupplier;
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

    public static DoubleUnaryOperator limitIntegralTermTo(DoubleSupplier limit) {
        return i -> Math.min(Math.abs(i), Math.abs(limit.getAsDouble())) * Math.signum(i);
    }

    protected int calculateError(int currentPosition, int targetPosition) {
        return currentPosition - targetPosition;
    }
    protected double calculateChangeInError(int currentError, int previousError, double deltaTime) {
        final double actualChange = (currentError - previousError) / deltaTime;
        return derivativeTermModifier.applyAsDouble(actualChange);
    }
    protected double calculateTotalError(int currentError, double previousTotalError) {
        final double actualTotal = currentError + previousTotalError;
        return integralTermModifier.applyAsDouble(actualTotal);
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

        double calculatedPower = (error * getProportionalCoefficient()) + (totalError * getIntegralCoefficient()) + (errorChange * getDerivativeCoefficient());
        return MOTOR_POWER_RANGE.clamp(calculatedPower);
    }

    public double getProportionalCoefficient() {
        return proportionalCoefficientSupplier.getAsDouble();
    }

    public double getIntegralCoefficient() {
        return integralCoefficientSupplier.getAsDouble();
    }

    public double getDerivativeCoefficient() {
        return derivativeCoefficientSupplier.getAsDouble();
    }
}
