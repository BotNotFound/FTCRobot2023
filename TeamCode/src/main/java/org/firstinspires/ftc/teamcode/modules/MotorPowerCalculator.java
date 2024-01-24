package org.firstinspires.ftc.teamcode.modules;

import android.util.Range;

public interface MotorPowerCalculator {
    Range<Double> MOTOR_POWER_RANGE = new Range<>(-1.0, 1.0);

    double calculateMotorPower(int currentMotorPosition, int targetMotorPosition);
}
