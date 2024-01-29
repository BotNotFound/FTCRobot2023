package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.modules.core.Module;

import java.util.concurrent.atomic.AtomicBoolean;

public class ActiveIntake extends Module {
	private final ConditionalHardwareDevice<DcMotor> intakeMotor;

	public static final String INTAKE_MOTOR_NAME = "Intake Motor";

	private double activePower = 0.7;

	private static final double TURBO_POWER = 1.0;

	private final AtomicBoolean on;

	public ActiveIntake(OpMode registrar) {
			super(registrar);
			on = new AtomicBoolean(false);
			intakeMotor = ConditionalHardwareDevice.tryGetHardwareDevice(parent.hardwareMap, DcMotor.class, INTAKE_MOTOR_NAME);

			intakeMotor.runIfAvailable(intake -> {
				intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				intake.setDirection(DcMotorSimple.Direction.REVERSE);
			});
	}

	public void setPower(double newPower) {
		activePower = newPower;
		intakeMotor.runIfAvailable(intake -> {
			if (on.get()) {
				intake.setPower(activePower);
			}
		});
	}

	public double getPower() {
		return activePower;
	}

	public void start() {
		if (on.compareAndSet(false, true)) {
			intakeMotor.runIfAvailable(intake ->
					intake.setPower(activePower)
			);
		}
	}

	public void reverse() {
		setPower(-getPower());
	}

	public void turbo() {
		on.set(true);
		intakeMotor.runIfAvailable(intake ->
			intake.setPower(TURBO_POWER)
		);
	}

	public void unTurbo() {
		setPower(getPower());
	}

	public boolean isTurbo() {
		return on.get() && intakeMotor.requireDevice().getPower() == TURBO_POWER;
	}

	public void stop() {
		if (on.compareAndSet(true, false)) {
			intakeMotor.runIfAvailable(intake ->
					intake.setPower(0.0)
			);
		}
	}

	public boolean isActive() {
		return on.get();
	}

	@Override
	public void cleanupModule() {
		stop();
	}

	@Override
	public void log() {
		intakeMotor.runIfAvailable(dcMotor -> {
			getTelemetry().addData("Is intake active", isActive());
			getTelemetry().addData("Intake power", getPower());
		});
	}
}
