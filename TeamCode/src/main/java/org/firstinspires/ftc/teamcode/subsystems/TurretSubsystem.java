package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Hardware and power control for a DC-motor-driven turret. */
public class TurretSubsystem {
    private final DcMotor turretMotor;

    public TurretSubsystem(HardwareMap hardwareMap) {
        // This must match the motor's name in the Robot Controller config.
        turretMotor = hardwareMap.get(DcMotor.class, "turret_motor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Sets turret rotation power, clipped to the legal motor range. */
    public void setPower(double power) {
        double clippedPower = Math.max(-1.0, Math.min(1.0, power));
        turretMotor.setPower(clippedPower);
    }

    public double getPower() {
        return turretMotor.getPower();
    }

    public void stop() {
        turretMotor.setPower(0);
    }
}