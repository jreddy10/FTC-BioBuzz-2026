package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

/** AprilTag turret aiming with operator-held flywheel control; no ball feeding. */
@TeleOp(name = "BioBuzz: AprilTag + Launcher", group = "BioBuzz")
public class BioBuzzAprilTagLauncherTeleOp extends BioBuzzAprilTagTeleOp {
    private static final double TARGET_RPM = 6000.0;

    private LauncherSubsystem launcher;

    @Override
    protected void initializeAdditionalHardware() {
        launcher = new LauncherSubsystem(hardwareMap);
        launcher.stop();
        telemetry.addLine("Hold left bumper to aim, then hold right bumper to spin at 6000 RPM");
        telemetry.addLine("Release either bumper or press B to stop the flywheel");
        telemetry.addLine("Keep clear of flywheel; no feeder control or shot-readiness detection");
    }

    @Override
    protected void updateAdditionalHardware(Gamepad controls, boolean aligned) {
        boolean requested = controls != null && controls.right_bumper;
        boolean spinning = requested && aligned && controls.left_bumper && !controls.b;
        if (spinning) {
            launcher.setVelocityRpm(TARGET_RPM);
        } else {
            launcher.stop();
        }

        telemetry.addData("Launcher", spinning ? "SPINNING"
                : requested ? "BLOCKED - hold aim bumper and align; release B" : "STOPPED");
        telemetry.addData("Launcher target RPM", "%.0f", spinning ? TARGET_RPM : 0.0);
        telemetry.addData("Launcher measured RPM", "%.0f", launcher.getVelocityRpm());
    }

    @Override
    protected void stopAdditionalHardware() {
        if (launcher != null) {
            launcher.stop();
        }
    }
}
