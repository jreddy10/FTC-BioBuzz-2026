package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Drive")
public class BioBuzzTeleOp extends LinearOpMode {

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private DcMotor intake_motor;

    private DcMotor flywheel;

    private DcMotor turret;

    private DcMotor viper;

        private Servo dual300Servo;

    private boolean viper_up_down = false;

    private GoBildaPinpointDriver pinpoint;

    private double currentFL = 0;
    private double currentFR = 0;
    private double currentBL = 0;
    private double currentBR = 0;

    private static final double POWER_STEP = 0.08;
    private static final double DEADZONE = 0.08;
        private static final double SERVO_BACKWARD_POSITION = 0.15;
        private static final double SERVO_FORWARD_POSITION = 0.85;
        private static final double SERVO_CENTER_POSITION = 0.50;
        private static final int SERVO_SWEEP_COUNT = 1;
        private static final long SERVO_MOVE_TIME_MS = 1200;

    @Override
    public void runOpMode() {

        frontLeftDrive =
                hardwareMap.get(DcMotor.class, "f_l_dr");
        backLeftDrive =
                hardwareMap.get(DcMotor.class, "b_l_dr");
        frontRightDrive =
                hardwareMap.get(DcMotor.class, "f_r_dr");
        backRightDrive =
                hardwareMap.get(DcMotor.class, "b_r_dr");
        turret =
                hardwareMap.get(DcMotor.class, "turret");
        intake_motor =
                hardwareMap.get(DcMotor.class, "intake_motor");
        flywheel =
                hardwareMap.get(DcMotor.class, "flywheel");
        viper =
                hardwareMap.get(DcMotor.class, "viper");
        dual300Servo =
                hardwareMap.get(Servo.class, "launch_servo");
        dual300Servo.setPosition(SERVO_CENTER_POSITION);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        DcMotor[] motors = {
                frontLeftDrive,
                backLeftDrive,
                frontRightDrive,
                backRightDrive
        };

        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE
            );
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

                turret.setPower(0);
                turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint"
        );

        // Retained from your original code.
        // Verify offsets and encoder directions for your mounting.
        pinpoint.setOffsets(7, 7, DistanceUnit.CM);

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods
                        .goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Keep the robot stationary during calibration.
        // Point it toward your intended field-forward direction.
        pinpoint.resetPosAndIMU();


        try {
            while (opModeInInit()) {
                pinpoint.update();

                telemetry.addLine("FIELD-RELATIVE DRIVE");
                telemetry.addLine("Keep robot stationary during calibration");
                telemetry.addData(
                        "Pinpoint status", pinpoint.getDeviceStatus()
                );
                addPinpointTelemetry();
                telemetry.update();

                sleep(20);
            }

            waitForStart();

            if (isStopRequested()) {
                return;
            }

                        // Gamepad 1 A starts one sweep; B cancels and centers the servo.
                        int servoState = 0; // 0 = idle, 1 = moving forward, 2 = moving backward
                        int servoCycle = 0;
                        long servoMoveStartMillis = 0;
                        boolean previousServoStart = false;
                        boolean previousServoCancel = false;

            while (opModeIsActive()) {
                                // Gamepad objects can be unavailable during initialization or
                                // reconnects. Treat missing input as neutral instead of crashing.
                                Gamepad driveGamepad = gamepad1;
                                Gamepad operatorGamepad = gamepad2;

                                double turretPower = operatorGamepad == null
                                                ? 0.0 : (operatorGamepad.left_stick_x)/4;
                                if (Math.abs(turretPower) < DEADZONE) {
                                        turretPower = 0;
                                }
                                turret.setPower(turretPower);

                                boolean servoStart = driveGamepad != null && driveGamepad.a;
                                boolean servoCancel = driveGamepad != null && driveGamepad.b;

                                if (servoStart && !previousServoStart && servoState == 0) {
                                        servoCycle = 1;
                                        servoState = 1;
                                        servoMoveStartMillis = System.currentTimeMillis();
                                        dual300Servo.setPosition(SERVO_FORWARD_POSITION);
                                }

                                if (servoCancel && !previousServoCancel && servoState != 0) {
                                        servoState = 0;
                                        servoCycle = 0;
                                        dual300Servo.setPosition(SERVO_CENTER_POSITION);
                                }

                                previousServoStart = servoStart;
                                previousServoCancel = servoCancel;

                                if (servoState != 0
                                                && System.currentTimeMillis() - servoMoveStartMillis
                                                >= SERVO_MOVE_TIME_MS) {
                                        if (servoState == 1) {
                                                servoState = 2;
                                                servoMoveStartMillis = System.currentTimeMillis();
                                                dual300Servo.setPosition(SERVO_BACKWARD_POSITION);
                                        } else if (servoCycle < SERVO_SWEEP_COUNT) {
                                                servoCycle++;
                                                servoState = 1;
                                                servoMoveStartMillis = System.currentTimeMillis();
                                                dual300Servo.setPosition(SERVO_FORWARD_POSITION);
                                        } else {
                                                servoState = 0;
                                                servoCycle = 0;
                                                dual300Servo.setPosition(SERVO_CENTER_POSITION);
                                        }
                                }

                                telemetry.addData(
                                                "Dual 300 servo test",
                                                servoState == 0 ? "Idle" : "Sweep %d/%d %s",
                                                servoCycle,
                                                SERVO_SWEEP_COUNT,
                                                servoState == 1 ? "forward" : "backward"
                                );
                                telemetry.addLine("Gamepad 1 A: start servo sweeps | B: cancel");

                pinpoint.update();

                // Do not drive with an unready heading sensor.
                if (pinpoint.getDeviceStatus()
                        != GoBildaPinpointDriver.DeviceStatus.READY) {
                    stopDrive();

                    telemetry.addData(
                            "Pinpoint not ready",
                            pinpoint.getDeviceStatus()
                    );
                    telemetry.update();

                    sleep(20);
                    continue;
                }
                if (gamepad2.a) {
                    intake_motor.setPower(1.0);
                }

                if (gamepad2.b) {
                    flywheel.setPower(1.0);
                }

                if (gamepad2.y && !viper_up_down) {
                    viper.setDirection(DcMotor.Direction.FORWARD);
                    viper.setPower(0.1);
                }
                
                if (gamepad2.y && viper_up_down) {
                    viper.setDirection(DcMotor.Direction.REVERSE);
                    viper.setPower(0.1);
                }


                double heading = pinpoint.getPosition()
                        .getHeading(AngleUnit.RADIANS);

                if (Double.isNaN(heading)
                        || Double.isInfinite(heading)) {
                    stopDrive();

                    telemetry.addLine("Invalid Pinpoint heading");
                    telemetry.update();

                    sleep(20);
                    continue;
                }

                double y = driveGamepad == null
                        ? 0.0 : -driveGamepad.left_stick_y;
                double x = driveGamepad == null
                        ? 0.0 : driveGamepad.left_stick_x;
                double yaw = driveGamepad == null
                        ? 0.0 : driveGamepad.right_stick_x;

                if (Math.abs(y) < DEADZONE) y = 0;
                if (Math.abs(x) < DEADZONE) x = 0;
                if (Math.abs(yaw) < DEADZONE) yaw = 0;

                if (x == 0 && y == 0 && yaw == 0) {
                    stopDrive();
                } else {
                    // Convert field-relative joystick commands
                    // into robot-relative movement.
                    double rotX =
                            x * Math.cos(heading)
                                    + y * Math.sin(heading);

                    double rotY =
                            -x * Math.sin(heading)
                                    + y * Math.cos(heading);

                    double fl = rotY + rotX + yaw;
                    double fr = rotY - rotX - yaw;
                    double bl = rotY - rotX + yaw;
                    double br = rotY + rotX - yaw;

                    double max = Math.max(
                            1.0,
                            Math.max(
                                    Math.max(Math.abs(fl), Math.abs(fr)),
                                    Math.max(Math.abs(bl), Math.abs(br))
                            )
                    );

                    fl /= max;
                    fr /= max;
                    bl /= max;
                    br /= max;

                    currentFL = moveTowards(currentFL, fl);
                    currentFR = moveTowards(currentFR, fr);
                    currentBL = moveTowards(currentBL, bl);
                    currentBR = moveTowards(currentBR, br);

                    frontLeftDrive.setPower(currentFL);
                    frontRightDrive.setPower(currentFR);
                    backLeftDrive.setPower(currentBL);
                    backRightDrive.setPower(currentBR);
                }

                telemetry.addLine("FIELD-RELATIVE DRIVE");
                addPinpointTelemetry();
                telemetry.update();

                sleep(20);
            }
        } finally {
            stopDrive();
                        turret.setPower(0);
                        dual300Servo.setPosition(SERVO_CENTER_POSITION);
        }
    }

    private void addPinpointTelemetry() {
        telemetry.addData(
                "Heading (degrees)",
                "%.1f",
                pinpoint.getPosition().getHeading(AngleUnit.DEGREES)
        );

        telemetry.addData(
                "X (mm)",
                "%.1f",
                pinpoint.getPosition().getX(DistanceUnit.MM)
        );

        telemetry.addData(
                "Y (mm)",
                "%.1f",
                pinpoint.getPosition().getY(DistanceUnit.MM)
        );
    }

    private void stopDrive() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        currentFL = 0;
        currentFR = 0;
        currentBL = 0;
        currentBR = 0;
    }

    private double moveTowards(double current, double target) {
        double difference = target - current;

        if (Math.abs(difference) <= POWER_STEP) {
            return target;
        }

        return current + Math.copySign(POWER_STEP, difference);
    }


}