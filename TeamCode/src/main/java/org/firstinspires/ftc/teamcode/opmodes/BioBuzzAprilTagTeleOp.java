/*
 * APRILTAG TURRET - SETUP AND TEST
 *
 * PURPOSE
 * - Tests only turret rotation; does not drive the robot or fire the launcher.
 * - Uses a Limelight 3A mounted on the goBILDA 2.75:1 gear-driven turret.
 * - The Limelight must rotate WITH the turret and point in its aiming azimuth.
 *   Upward camera pitch is allowed; keep sideways roll near zero. This aims at
 *   the tag center horizontally, not an offset scoring point.
 *
 * SETUP
 * 1. Connect the turret DC motor to a Control/Expansion Hub motor port and the
 *    Limelight to the Control Hub by USB. Configure their names exactly as:
 *       turret    (DC motor)
 *       limelight (Limelight 3A, not a webcam)
 * 2. In the Limelight web interface, configure pipeline 0 for AprilTag detection
 *    with tag family 36h11 and tag size 3.25 inches for accurate pose estimates.
 *    Place a red CELL tag (ID 30-37) in view and confirm it is
 *    detected. APRILTAG_PIPELINE must match that pipeline's index.
 * 3. Build/install the updated Robot Controller app using your FTC workflow.
 *    Select "BioBuzz: AprilTag Turret" on the Driver Station.
 * 4. Secure the robot, disable/remove launcher feed, and keep hands, clothing,
 *    and loose wires away from moving gears. Leave clearance for turret cables.
 *
 * GAMEPAD 1 CONTROLS
 * - Left stick left/right: manual turret rotation (maximum 20%, within +/-60 deg).
 * - With left bumper RELEASED: D-pad left selects the opposite-audience CELL
 *   (30-33); D-pad right selects the audience CELL (34-37); D-pad down selects
 *   both red CELLS (default). Selection does not determine which CELL faces up.
 * - HOLD left bumper: sweep within +/-50 degrees, then aim at a visible tag.
 * - Release left bumper: manual control resumes; center the stick to stop.
 * - HOLD B: stop override in either mode. Releasing B resumes normal controls.
 * - Driver Station STOP: end the OpMode and stop the turret and Limelight polling.
 *
 * FIRST TEST
 * 1. Press INIT, then START with the stick centered and bumper released.
 * 2. Briefly nudge the stick right. The turret should turn right when viewed
 *    from behind the aiming axis. If reversed, STOP, change
 *    TURRET_DIRECTION_SIGN from 1.0 to -1.0, and rebuild/install before retesting.
 * 3. Put one tag slightly off-center in the camera view. Check telemetry for its
 *    Tag ID and horizontal aim error, then briefly hold the left bumper.
 * 4. The aim error should decrease. If it grows, stop immediately and check
 *    motor direction, camera orientation, and mounting before another test.
 * 5. Keep holding the bumper: power decreases toward the tag center and becomes
 *    zero within +/-2 degrees (ALIGNED). It corrects again if the error increases.
 * 6. Move the tag out of view: bounded scanning resumes if detection is missing,
 *    invalid, or older than 250 ms.
 *
 * TARGET SELECTION AND LIMITATIONS
 * - Both red CELLS are eligible (30-33 and 34-37); blue tags are ignored.
 * - DESIRED_TAG_ID = -1 acquires the first eligible visible tag.
 *   If it disappears, scanning resumes and another eligible tag can be acquired.
 * - All fresh visible red tags show tx/ty, camera-space XYZ in inches, and
 *   pitch/roll/yaw in degrees. These are relative to the CAMERA, not the floor.
 * - Hive facing state is UNKNOWN: no unverified tilt threshold is used, and
 *   ALIGNED means horizontal tag alignment ONLY, not permission to launch.
 *
 * HIVE POSE TEST (NO LAUNCHING)
 * 1. Keep the robot and turret stationary. On a practice FIELD, observe each red
 *    CELL in both stable HIVE positions. Record its tag IDs, XYZ and orientation,
 *    along with which opening physically faces upward. Use authorized FIELD
 *    handling procedures; do not manipulate a competition HIVE during a MATCH.
 * 2. Repeat from several distances and robot/turret angles. Camera-relative pitch
 *    alone is not a universal Hive tilt measurement: mounting/robot pose matter.
 * 3. Select the physically upward-facing CELL with the D-pad, then test aiming.
 *    After a TIP, release the bumper and select the other CELL before re-aiming.
 * 4. Launch readiness needs verified cell orientation, a tag-to-opening offset,
 *    distance/elevation calibration and flywheel speed checks; none are assumed.
 * - Center the turret before INIT. Encoder-based software limits are +/-60
 *   degrees, based on the 5202 motor and 2.75:1 turret reduction.
 * - Software limits are not physical stops or a substitute for checking wiring.
 *   Encoder counts cannot detect slipping gears or an incorrectly centered start.
 * - A nonzero power reading is a command, not proof that the motor is moving.
 *
 * UPWARD CAMERA MOUNT - MEASURE BEFORE FIELD TUNING
 * - Record camera pitch above horizontal (degrees), lens-center height above
 *   the TILE surface, and lens offsets from the turret pivot (forward/side/up).
 * - Secure the mount; it must rotate with the turret without flexing. Keep
 *   camera roll near zero: sideways tilt couples horizontal and vertical errors.
 * - This code uses tx for horizontal aiming. It does NOT apply camera pitch,
 *   height, or mounting-offset corrections to the displayed camera-space pose.
 * - Camera-relative pitch/roll/yaw are NOT directly the HIVE's field tilt.
 *   Future height/tilt calculations need a verified camera-to-field transform,
 *   including robot orientation and turret angle. Recalibrate after mount changes.
 *
 * FIELD TUNING CHECKLIST (PRACTICE FIELD, LAUNCHER INACTIVE)
 * 1. Confirm AprilTag pipeline 0, family 36h11, and physical tag size 3.25 inches.
 *    Test detection with the robot stationary before enabling turret movement.
 * 2. Tune Limelight exposure/gain for clear tags under field lighting. Shorter
 *    exposure reduces motion blur but may require more gain/light. Excess gain
 *    adds noise. Verify detection both stationary and during slow turret motion.
 * 3. Verify TURRET_DIRECTION_SIGN (currently 1.0): nudging the LEFT stick right
 *    should rotate the turret right. If auto aim turns away from
 *    a tag, stop immediately; verify mounting/sign before tuning gain.
 * 4. TURN_GAIN starts at 0.01 power per degree. If it overshoots or oscillates,
 *    decrease it in small steps. If response is too slow, increase slightly,
 *    changing one setting at a time and checking several starting aim errors.
 * 5. MAX_AUTO_POWER and MANUAL_POWER start at 0.20. Keep these low for first
 *    tests. If the mechanism does not move, check wiring/binding first; do not
 *    increase power against a hard stop. Any increase needs a controlled retest.
 * 6. AIM_TOLERANCE_DEGREES starts at 2.0. Tighten only after tracking is stable;
 *    a tolerance below camera noise/backlash can cause hunting. If it stops
 *    outside tolerance, near-zero proportional power may not overcome friction.
 * 7. DEADBAND (0.08) affects manual stick drift only, not automatic aiming.
 *    MAX_RESULT_AGE_MS (250) stops stale-result auto aim; do not increase it to
 *    conceal poor detection or communications. Check B and lost-tag stopping.
 * 8. Repeat in BOTH stable HIVE positions at several distances and robot/turret
 *    angles. Record tag ID, tx/ty, XYZ, pitch/roll/yaw, actual upward-facing CELL,
 *    camera mounting measurements, and the gain/power/tolerance used each time.
 * 9. Upward pitch alone does not require changing TURN_GAIN or the 2.75:1 ratio.
 *    Horizontal ALIGNED is not launch-ready: verify the CELL opening offset,
 *    field-relative pose, launcher elevation/distance and flywheel speed first.
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/** Turret-only manual rotation and hold-to-aim Limelight AprilTag tracking. */
@TeleOp(name = "BioBuzz: AprilTag Turret", group = "BioBuzz")
public class BioBuzzAprilTagTeleOp extends LinearOpMode {
    // Requires the camera to rotate WITH the turret and point along its aim axis.
    // Software limits depend on a centered startup and the verified encoder gearing.
    // Configure pipeline 0 as an AprilTag pipeline in the Limelight web interface.
    private static final int APRILTAG_PIPELINE = 0;
    private static final int RED_CELL_MIN_ID = 30;
    private static final int RED_CELL_MAX_ID = 37;
    private static final int DESIRED_TAG_ID = -1; // -1: acquire the first eligible visible red tag
    private static final double TURN_GAIN = 0.01;
    private static final double AIM_TOLERANCE_DEGREES = 2.0;
    // Change to -1.0 if positive motor power turns the turret LEFT instead of RIGHT.
    private static final double TURRET_DIRECTION_SIGN = 1.0;
    private static final double MAX_AUTO_POWER = 0.20;
    private static final double MANUAL_POWER = 0.20;
    private static final double DEADBAND = 0.08;
    private static final double MAX_RESULT_AGE_MS = 250.0;

    @Override
    public void runOpMode() throws InterruptedException {
        TurretSubsystem turret = new TurretSubsystem(hardwareMap);

        Limelight3A limelight = null;
        try {
            initializeAdditionalHardware();
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(50);
            limelight.pipelineSwitch(APRILTAG_PIPELINE);
            limelight.start();

            telemetry.addLine("Center turret before INIT; encoder zero is set at INIT");
            telemetry.addLine("Gamepad 1 left stick X: manual rotation within +/-60 deg");
            telemetry.addLine("Hold left bumper: scan within +/-50 deg, then aim at RED CELL");
            telemetry.addLine("Bumper released: D-pad left 30-33 | right 34-37 | down both");
            telemetry.addLine("Hive facing state UNKNOWN - no launch-ready detection");
            telemetry.addLine("Limelight 3A: AprilTag pipeline 0, family 36h11");
            telemetry.addLine("B: stop override | Release bumper: manual control");
            telemetry.addLine("Encoder software limit +/-60 deg; verify cable clearance.");
            telemetry.update();
            waitForStart();

            Integer lockedId = null;
            boolean previousAssist = false;
            int selectedCell = 0; // 0 = both, 1 = opposite audience, 2 = audience
            double scanDirection = 1.0;

            while (opModeIsActive()) {
                Gamepad controls = gamepad1;
                boolean assist = controls != null && controls.left_bumper;
                boolean stop = controls == null || controls.b;
                if (!assist && controls != null) {
                    if (controls.dpad_left) selectedCell = 1;
                    if (controls.dpad_right) selectedCell = 2;
                    if (controls.dpad_down) selectedCell = 0;
                }
                if (!assist || !previousAssist) {
                    lockedId = null;
                }
                previousAssist = assist;

                LLResult result = limelight.getLatestResult();
                LLResultTypes.FiducialResult target = null;
                double horizontalOffset = 0;
                boolean fresh = result != null && result.isValid()
                        && result.getPipelineIndex() == APRILTAG_PIPELINE
                        && Double.isFinite(result.getStaleness())
                        && result.getStaleness() >= 0
                        && result.getStaleness() <= MAX_RESULT_AGE_MS;
                if (fresh && result.getFiducialResults() != null) {
                    for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                        int id = fiducial.getFiducialId();
                        if (id < RED_CELL_MIN_ID || id > RED_CELL_MAX_ID) {
                            continue;
                        }
                        addCellPoseTelemetry(fiducial);
                        if (target != null
                            || (selectedCell == 1 && id > 33)
                            || (selectedCell == 2 && id < 34)
                            || (DESIRED_TAG_ID >= 0 && id != DESIRED_TAG_ID)
                            || (lockedId != null && id != lockedId)) {
                            continue;
                        }
                        double tx = fiducial.getTargetXDegrees();
                        if (!Double.isFinite(tx)) {
                            continue;
                        }
                        target = fiducial;
                        horizontalOffset = tx;
                        if (assist) {
                            lockedId = id;
                        }
                    }
                }

                boolean usable = target != null;
                double turretPower = 0;
                String mode = "STOPPED";
                if (!stop && assist) {
                    if (usable) {
                        mode = "AUTO AIM";
                        // Positive Limelight tx means the tag is to the right.
                        turretPower = TURRET_DIRECTION_SIGN * correction(horizontalOffset,
                                TURN_GAIN, AIM_TOLERANCE_DEGREES);
                        if (Math.abs(horizontalOffset) <= AIM_TOLERANCE_DEGREES) {
                            mode = "ALIGNED";
                        }
                    } else {
                        lockedId = null;
                        if (turret.isAtScanLimit(scanDirection)) {
                            scanDirection *= -1.0;
                        }
                        mode = scanDirection > 0 ? "SCANNING RIGHT" : "SCANNING LEFT";
                        turretPower = scanDirection * 0.10;
                    }
                } else if (!stop) {
                    mode = "MANUAL TURRET";
                    turretPower = TURRET_DIRECTION_SIGN
                            * neutral(controls.left_stick_x) * MANUAL_POWER;
                }

                double appliedPower = turret.setPower(turretPower);
                if (Math.abs(appliedPower - turretPower) > 0.001 && !stop) {
                    mode = "SOFT LIMIT";
                }

                updateAdditionalHardware(controls, "ALIGNED".equals(mode));

                telemetry.addData("Mode", mode);
                telemetry.addData("Selected CELL", selectedCell == 1 ? "Opposite audience: 30-33"
                    : selectedCell == 2 ? "Audience: 34-37" : "Both RED CELLS: 30-37");
                telemetry.addData("Hive facing state", "UNKNOWN - verify physically");
                telemetry.addData("Launch readiness", "NOT IMPLEMENTED (alignment only)");
                telemetry.addData("Camera", "Limelight 3A (pipeline %d)", APRILTAG_PIPELINE);
                telemetry.addData("Result age (ms)", result == null ? "No data"
                    : String.format("%d", result.getStaleness()));
                if (usable) {
                        telemetry.addData("Tag ID", target.getFiducialId());
                    telemetry.addData("Horizontal aim error", "%.1f deg", horizontalOffset);
                }
                telemetry.addData("Turret angle (deg)", "%.1f", turret.getAngleDegrees());
                telemetry.addData("Turret encoder", turret.getEncoderPosition());
                telemetry.addData("Turret power", "%.2f", appliedPower);
                telemetry.update();
                sleep(20);
            }
        } finally {
            try {
                stopAdditionalHardware();
            } finally {
                turret.setPower(0);
                if (limelight != null) {
                    limelight.stop();
                }
            }
        }
    }

    protected void initializeAdditionalHardware() {
    }

    protected void updateAdditionalHardware(Gamepad controls, boolean aligned) {
    }

    protected void stopAdditionalHardware() {
    }

    private static double neutral(double value) {
        return Math.abs(value) < DEADBAND ? 0 : value;
    }

        private void addCellPoseTelemetry(LLResultTypes.FiducialResult tag) {
        String label = "Red tag " + tag.getFiducialId();
        telemetry.addData(label + " CELL", tag.getFiducialId() <= 33
            ? "Opposite audience" : "Audience");
        telemetry.addData(label + " tx / ty", "%.1f / %.1f deg",
            tag.getTargetXDegrees(), tag.getTargetYDegrees());
        Pose3D pose = tag.getTargetPoseCameraSpace();
        if (pose == null || pose.getPosition() == null || pose.getOrientation() == null) {
            telemetry.addData(label + " pose", "Unavailable");
            return;
        }
        Position position = pose.getPosition().toUnit(DistanceUnit.INCH);
        telemetry.addData(label + " camera XYZ", "%.1f / %.1f / %.1f in",
            position.x, position.y, position.z);
        telemetry.addData(label + " camera pitch / roll / yaw", "%.1f / %.1f / %.1f deg",
            pose.getOrientation().getPitch(AngleUnit.DEGREES),
            pose.getOrientation().getRoll(AngleUnit.DEGREES),
            pose.getOrientation().getYaw(AngleUnit.DEGREES));
        }

    private static double correction(double error, double gain, double tolerance) {
        return Math.abs(error) <= tolerance ? 0
                : Range.clip(error * gain, -MAX_AUTO_POWER, MAX_AUTO_POWER);
    }
}