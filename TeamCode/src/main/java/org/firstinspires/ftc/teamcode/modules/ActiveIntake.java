package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.modules.core.Module;

public class ActiveIntake extends Module {
	private final ConditionalHardwareDevice<DcMotor> intakeMotor;

	public static final String INTAKE_MOTOR_NAME = "Intake Motor";

	public ActiveIntake(OpMode registrar) {
			super(registrar);
			intakeMotor = ConditionalHardwareDevice.tryGetHardwareDevice(parent.hardwareMap, DcMotor.class, INTAKE_MOTOR_NAME);
	}

	public void start() {
		intakeMotor.runIfAvailable(intake ->
			intake.setPower(-1.0)
		);
	}

	public void stop() {
		intakeMotor.runIfAvailable(intake ->
				intake.setPower(0.0)
		);
	}

	@Override
	public void cleanupModule() {
		stop();
	}

	@Override
	public void log() {}
}
