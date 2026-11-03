package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Hardware and power control for a DC-motor-driven turret. */
public class TurretSubsystem {
    private static final double MOTOR_TICKS_PER_OUTPUT_REVOLUTION = 1425.1;
    private static final double MOTOR_REVOLUTIONS_PER_TURRET_REVOLUTION = 2.75;
    private static final double TICKS_PER_TURRET_DEGREE =
            MOTOR_TICKS_PER_OUTPUT_REVOLUTION
                    * MOTOR_REVOLUTIONS_PER_TURRET_REVOLUTION / 360.0;
    private static final int MAX_TURRET_TICKS =
            (int) (TICKS_PER_TURRET_DEGREE * 60.0);
    private static final int SCAN_LIMIT_TICKS =
            (int) (TICKS_PER_TURRET_DEGREE * 50.0);
    private static final int APPROACH_ZONE_TICKS =
            MAX_TURRET_TICKS - SCAN_LIMIT_TICKS;

    private final DcMotor turretMotor;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        turretMotor.setPower(0);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Sets power with a tapered approach to the encoder-based +/-60 degree limit. */
    public double setPower(double power) {
        double clippedPower = Math.max(-1.0, Math.min(1.0, power));
        int position = turretMotor.getCurrentPosition();

        if (clippedPower > 0) {
            if (position >= MAX_TURRET_TICKS) {
                clippedPower = 0;
            } else if (position > SCAN_LIMIT_TICKS) {
                clippedPower *= (double) (MAX_TURRET_TICKS - position)
                        / APPROACH_ZONE_TICKS;
            }
        } else if (clippedPower < 0) {
            if (position <= -MAX_TURRET_TICKS) {
                clippedPower = 0;
            } else if (position < -SCAN_LIMIT_TICKS) {
                clippedPower *= (double) (MAX_TURRET_TICKS + position)
                        / APPROACH_ZONE_TICKS;
            }
        }

        turretMotor.setPower(clippedPower);
        return clippedPower;
    }

    public double getPower() {
        return turretMotor.getPower();
    }

    public int getEncoderPosition() {
        return turretMotor.getCurrentPosition();
    }

    public double getAngleDegrees() {
        return getEncoderPosition() / TICKS_PER_TURRET_DEGREE;
    }

    public boolean isAtScanLimit(double direction) {
        int position = getEncoderPosition();
        return direction > 0
                ? position >= SCAN_LIMIT_TICKS
                : position <= -SCAN_LIMIT_TICKS;
    }

    public void stop() {
        turretMotor.setPower(0);
    }
}