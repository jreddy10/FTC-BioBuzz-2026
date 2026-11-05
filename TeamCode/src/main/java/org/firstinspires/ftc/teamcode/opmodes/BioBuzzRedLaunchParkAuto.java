package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/** Stationary timed launch with intake assist, then a short red-alliance parking route. */
@Autonomous(name = "BioBuzz: Red Launch + Park", group = "BioBuzz")
public class BioBuzzRedLaunchParkAuto extends LinearOpMode {
    private static final double TARGET_RPM = 6000.0;
    private static final double RPM_TOLERANCE = 200.0;
    private static final double AIM_TOLERANCE_DEGREES = 2.0;
    private static final double TURRET_DIRECTION_SIGN = 1.0;
    private static final double READY_HOLD_SECONDS = 0.5;
    private static final double AIM_TIMEOUT_SECONDS = 8.0;
    private static final double LAUNCH_SECONDS = 6.0;
    private static final double INTAKE_POWER = IntakeSubsystem.DEFAULT_POWER;
    private static final double FORWARD_INCHES = 4.0;
    private static final double RIGHT_INCHES = 8.0;
    private static final double MAX_DRIVE_POWER = 0.25;
    private static final double POSITION_TOLERANCE_INCHES = 0.35;
    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(2.0);
    private static final double MOVE_TIMEOUT_SECONDS = 4.0;
    private static final int APRILTAG_PIPELINE = 0;

    private final DcMotor[] driveMotors = new DcMotor[4];
    private TurretSubsystem turret;
    private LauncherSubsystem launcher;
    private DcMotor intake;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    private Integer lockedTagId;
    private double scanDirection = 1.0;
    private String launchOutcome = "Not started";
    private String routeOutcome = "Not started";

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initializeHardware();
            while (opModeInInit()) {
                pinpoint.update();
                telemetry.addLine("RED: stationary launch, forward 4 in, right 8 in");
                telemetry.addLine("Center turret before INIT; keep robot still for calibration");
                telemetry.addLine("Intake runs only during timed launch; no ball counter");
                telemetry.addLine("Verify upward-facing CELL 34-37 and calibrated wall shot");
                telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
                telemetry.update();
                sleep(20);
            }
            waitForStart();
            if (!opModeIsActive()) {
                return;
            }

            Pose2D start = readPose();
            if (start == null) {
                return;
            }
            aimAndLaunch();
            stopLaunchMechanisms();
            turret.stop();
            if (!opModeIsActive()) {
                return;
            }

            double heading = start.getHeading(AngleUnit.RADIANS);
            double forwardX = start.getX(DistanceUnit.INCH) + FORWARD_INCHES * Math.cos(heading);
            double forwardY = start.getY(DistanceUnit.INCH) + FORWARD_INCHES * Math.sin(heading);
            if (!moveTo("CLEAR WALL", forwardX, forwardY, heading)) {
                return;
            }
            if (!moveTo("PARK RIGHT",
                    forwardX + RIGHT_INCHES * Math.sin(heading),
                    forwardY - RIGHT_INCHES * Math.cos(heading), heading)) {
                return;
            }
            routeOutcome = "Parking complete";
        } finally {
            try {
                stopLaunchMechanisms();
            } finally {
                try {
                    if (turret != null) {
                        turret.stop();
                    }
                } finally {
                    stopDrive();
                    if (limelight != null) {
                        limelight.stop();
                    }
                }
            }
            telemetry.addData("Launch result", launchOutcome);
            telemetry.addData("Route result", routeOutcome);
            telemetry.addLine("All commanded motors stopped; flywheel coasts");
            telemetry.update();
        }
    }

    private void initializeHardware() {
        String[] names = {"f_l_dr", "f_r_dr", "b_l_dr", "b_r_dr"};
        for (int i = 0; i < driveMotors.length; i++) {
            driveMotors[i] = hardwareMap.get(DcMotor.class, names[i]);
            driveMotors[i].setPower(0);
            driveMotors[i].setDirection(i == 0 || i == 2
                    ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        turret = new TurretSubsystem(hardwareMap);
        launcher = new LauncherSubsystem(hardwareMap);
        launcher.stop();
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        intake.setPower(0);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(80, 25.4, DistanceUnit.MM);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
    }

    private void aimAndLaunch() throws InterruptedException {
        stopDrive();
        ElapsedTime phase = new ElapsedTime();
        ElapsedTime stable = new ElapsedTime();
        boolean launching = false;
        boolean previouslyReady = false;
        while (opModeIsActive()) {
            boolean aligned = updateAim(!launching);
            double rpm = launcher.getVelocityRpm();
            boolean speedReady = Double.isFinite(rpm)
                    && Math.abs(rpm - TARGET_RPM) <= RPM_TOLERANCE;
            if (launching && !aligned) {
                launchOutcome = "Alignment lost during launch; skipped remainder";
                return;
            }
            if (!Double.isFinite(rpm)) {
                launchOutcome = "Invalid shooter RPM; skipped launch";
                return;
            }
            launcher.setVelocityRpm(TARGET_RPM);
            if (!launching) {
                boolean ready = aligned && speedReady;
                if (!ready || !previouslyReady) {
                    stable.reset();
                }
                previouslyReady = ready;
                if (ready && stable.seconds() >= READY_HOLD_SECONDS) {
                    launching = true;
                    phase.reset();
                } else if (phase.seconds() >= AIM_TIMEOUT_SECONDS) {
                    launchOutcome = "Aim/RPM timeout; skipped launch";
                    return;
                }
            } else if (phase.seconds() >= LAUNCH_SECONDS) {
                launchOutcome = "Six-second launch window complete; ball count unverified";
                return;
            }
            intake.setPower(launching ? INTAKE_POWER : 0);
            telemetry.addData("Intake", launching ? "ON" : "OFF");
            telemetry.addData("Phase", launching ? "TIMED LAUNCH" : "AIM + SPIN UP");
            telemetry.addData("Phase seconds", "%.1f", phase.seconds());
            telemetry.addData("Shooter RPM / target", "%.0f / %.0f", rpm, TARGET_RPM);
            telemetry.addData("Speed within tolerance", speedReady);
            telemetry.addData("Stable seconds", "%.2f", stable.seconds());
            telemetry.update();
            sleep(20);
        }
        launchOutcome = "Stopped by Driver Station";
    }

    private boolean updateAim(boolean allowScan) {
        LLResult result = limelight.getLatestResult();
        LLResultTypes.FiducialResult target = null;
        if (result != null && result.isValid() && result.getPipelineIndex() == APRILTAG_PIPELINE
                && Double.isFinite(result.getStaleness()) && result.getStaleness() >= 0
                && result.getStaleness() <= 250 && result.getFiducialResults() != null) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                if (id >= 34 && id <= 37 && (lockedTagId == null || lockedTagId == id)
                        && Double.isFinite(tag.getTargetXDegrees())) {
                    target = tag;
                    break;
                }
            }
        }
        if (target == null) {
            lockedTagId = null;
            if (allowScan) {
                if (turret.isAtScanLimit(scanDirection)) {
                    scanDirection *= -1.0;
                }
                turret.setPower(scanDirection * 0.10);
            } else {
                turret.stop();
            }
            telemetry.addData("Aim", allowScan ? "SCANNING (34-37)" : "TARGET LOST");
            return false;
        }
        boolean sameTarget = lockedTagId != null;
        lockedTagId = target.getFiducialId();
        double error = target.getTargetXDegrees();
        boolean aligned = Math.abs(error) <= AIM_TOLERANCE_DEGREES;
        turret.setPower(aligned ? 0
                : TURRET_DIRECTION_SIGN * Range.clip(error * 0.01, -0.20, 0.20));
        telemetry.addData("Tag ID", lockedTagId);
        telemetry.addData("Aim error (deg)", "%.1f", error);
        telemetry.addData("Turret angle (deg)", "%.1f", turret.getAngleDegrees());
        // A newly acquired tag must start a new continuous readiness interval.
        return aligned && sameTarget;
    }

    private Pose2D readPose() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                || pose == null || !Double.isFinite(pose.getX(DistanceUnit.INCH))
                || !Double.isFinite(pose.getY(DistanceUnit.INCH))
                || !Double.isFinite(pose.getHeading(AngleUnit.RADIANS))) {
            stopDrive();
            routeOutcome = "Aborted: Pinpoint not READY or invalid pose";
            telemetry.addData("Parking error", routeOutcome);
            telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
            telemetry.update();
            return null;
        }
        return pose;
    }

    private boolean moveTo(String label, double targetX, double targetY, double targetHeading)
            throws InterruptedException {
        ElapsedTime timeout = new ElapsedTime();
        ElapsedTime settled = new ElapsedTime();
        boolean previouslyAtTarget = false;
        try {
            while (opModeIsActive() && timeout.seconds() < MOVE_TIMEOUT_SECONDS) {
                Pose2D pose = readPose();
                if (pose == null) {
                    return false;
                }
                double heading = pose.getHeading(AngleUnit.RADIANS);
                double errorX = targetX - pose.getX(DistanceUnit.INCH);
                double errorY = targetY - pose.getY(DistanceUnit.INCH);
                double distance = Math.hypot(errorX, errorY);
                double headingError = Math.atan2(Math.sin(targetHeading - heading),
                        Math.cos(targetHeading - heading));
                boolean positionReached = distance <= POSITION_TOLERANCE_INCHES;
                boolean headingReached = Math.abs(headingError) <= HEADING_TOLERANCE_RADIANS;
                if (positionReached && headingReached) {
                    stopDrive();
                    if (!previouslyAtTarget) {
                        settled.reset();
                    }
                    previouslyAtTarget = true;
                    if (settled.seconds() >= 0.25) {
                        return true;
                    }
                } else {
                    previouslyAtTarget = false;
                    settled.reset();
                    double power = positionReached ? 0
                            : Range.clip(distance * 0.08, 0.10, MAX_DRIVE_POWER);
                    // Pinpoint X is forward and Y is left; mecanum lateral power is right.
                    double forward = positionReached ? 0
                            : (errorX * Math.cos(heading) + errorY * Math.sin(heading))
                                    / distance * power;
                    double right = positionReached ? 0
                            : (errorX * Math.sin(heading) - errorY * Math.cos(heading))
                                    / distance * power;
                    double yaw = headingReached ? 0 : -Range.clip(headingError, -0.15, 0.15);
                    setDrive(forward, right, yaw);
                }
                telemetry.addData("Phase", label);
                telemetry.addData("Launch result", launchOutcome);
                telemetry.addData("Distance remaining (in)", "%.2f", distance);
                telemetry.addData("Heading error (deg)", "%.1f", Math.toDegrees(headingError));
                telemetry.update();
                sleep(20);
            }
            routeOutcome = opModeIsActive() ? label + ": movement timeout"
                    : "Stopped by Driver Station";
            return false;
        } finally {
            stopDrive();
        }
    }

    private void setDrive(double forward, double right, double yaw) {
        double[] powers = {forward + right + yaw, forward - right - yaw,
                forward - right + yaw, forward + right - yaw};
        double scale = MAX_DRIVE_POWER;
        for (double power : powers) {
            scale = Math.max(scale, Math.abs(power));
        }
        for (int i = 0; i < driveMotors.length; i++) {
            driveMotors[i].setPower(powers[i] * MAX_DRIVE_POWER / scale);
        }
    }

    private void stopLaunchMechanisms() {
        try {
            if (intake != null) {
                intake.setPower(0);
            }
        } finally {
            if (launcher != null) {
                launcher.stop();
            }
        }
    }

    private void stopDrive() {
        for (DcMotor motor : driveMotors) {
            if (motor != null) {
                motor.setPower(0);
            }
        }
    }
}
