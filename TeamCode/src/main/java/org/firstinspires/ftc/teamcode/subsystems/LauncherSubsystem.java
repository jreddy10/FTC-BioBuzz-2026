package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Encoder-based velocity control for a single-motor launcher flywheel. */
public class LauncherSubsystem {
    private static final double SECONDS_PER_MINUTE = 60.0;
    private static final double DEGREES_PER_REVOLUTION = 360.0;

    private final DcMotorEx launcherMotor;

    public LauncherSubsystem(HardwareMap hardwareMap) {
        // Configure this motor as a DcMotorEx with its correct encoder type in the RC.
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Commands flywheel speed in RPM using the motor's encoder feedback. */
    public void setVelocityRpm(double rpm) {
        if (rpm < 0) {
            throw new IllegalArgumentException("Launcher RPM must not be negative");
        }

        double degreesPerSecond = rpm * DEGREES_PER_REVOLUTION / SECONDS_PER_MINUTE;
        launcherMotor.setVelocity(degreesPerSecond, AngleUnit.DEGREES);
    }

    /** Returns the measured flywheel speed in RPM. */
    public double getVelocityRpm() {
        double degreesPerSecond = launcherMotor.getVelocity(AngleUnit.DEGREES);
        return degreesPerSecond * SECONDS_PER_MINUTE / DEGREES_PER_REVOLUTION;
    }

    public void stop() {
        launcherMotor.setPower(0);
    }
}