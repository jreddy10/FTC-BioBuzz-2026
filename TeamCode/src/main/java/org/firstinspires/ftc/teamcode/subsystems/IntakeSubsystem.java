package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Hardware and power control for the intake motor. */
public class IntakeSubsystem {
    private final DcMotor intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        // This must exactly match the motor's name in the Robot Controller config.
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Sets intake motor power; positive and negative directions depend on motor mounting. */
    public void setPower(double power) {
        double clippedPower = Math.max(-1.0, Math.min(1.0, power));
        intakeMotor.setPower(clippedPower);
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}