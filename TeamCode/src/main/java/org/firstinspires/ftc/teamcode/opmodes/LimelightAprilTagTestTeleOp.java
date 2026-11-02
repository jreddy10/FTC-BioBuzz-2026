package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

/** Camera-only test: verifies Limelight 3A AprilTag detections without a turret. */
@TeleOp(name = "BioBuzz: Limelight Tag Test", group = "BioBuzz")
public class LimelightAprilTagTestTeleOp extends LinearOpMode {
    private static final String LIMELIGHT_NAME = "limelight";
    private static final int APRILTAG_PIPELINE = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addData("Camera", "Limelight 3A: %s", LIMELIGHT_NAME);
        telemetry.addData("Pipeline", APRILTAG_PIPELINE);
        telemetry.addLine("Configure this pipeline for AprilTags (36h11).");
        telemetry.addLine("Press START to view live detections; STOP to exit.");
        telemetry.update();

        try {
            waitForStart();
            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                telemetry.addData("Pipeline", APRILTAG_PIPELINE);

                if (result == null) {
                    telemetry.addLine("Waiting for Limelight result...");
                } else {
                    telemetry.addData("Valid result", result.isValid());
                    telemetry.addData("Result pipeline", result.getPipelineIndex());
                    telemetry.addData("Result age (ms)", "%d", result.getStaleness());

                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    int count = fiducials == null ? 0 : fiducials.size();
                    telemetry.addData("AprilTag count", count);

                    if (result.isValid() && count > 0) {
                        for (LLResultTypes.FiducialResult fiducial : fiducials) {
                            telemetry.addData(
                                    "Tag " + fiducial.getFiducialId() + " tx/ty",
                                    "%.1f / %.1f deg",
                                    fiducial.getTargetXDegrees(),
                                    fiducial.getTargetYDegrees()
                            );
                        }
                    } else {
                        telemetry.addLine("No AprilTags detected in this result.");
                    }
                }

                telemetry.update();
                sleep(50);
            }
        } finally {
            limelight.stop();
        }
    }
}
