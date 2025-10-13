/*
 * Copyright (c) 2025 FIRST
 *
 * Enhanced StarterBot TeleOp with:
 * - Clipped motor powers to avoid drift
 * - Drivetrain motors set to RUN_WITHOUT_ENCODER
 * - Analog slow-mode using left trigger
 * - Battery voltage compensation for launcher velocity
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Enhanced StarterBot TeleOp v2.1", group = "StarterBot")
public class EnhancedStarterBot_TeleOp extends OpMode {

    // --- Constants ---
    private final double FEED_TIME_SECONDS = 0.25;
    private final double FEEDER_POWER = 1.0;
    private final double LOW_VELOCITY = 1000;
    private final double HIGH_VELOCITY = 1200;
    private final double VELOCITY_TOLERANCE = 25;

    // Reference voltage for compensation (typical FTC battery)
    private final double NOMINAL_VOLTAGE = 13.0;

    // Mecanum geometry
    private final double wheelX = 0.15;
    private final double wheelY = 0.15;

    // --- Hardware ---
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;

    // --- Timers ---
    private ElapsedTime feederTimer = new ElapsedTime();

    // --- Launcher State Machine ---
    private enum LaunchState { IDLE, WAITING_FOR_SPEED, FEEDING }
    private LaunchState launchState = LaunchState.IDLE;

    private double currentTargetVelocity = 0;
    private boolean flywheelAtSpeed = false;

    @Override
    public void init() {
        // --- Hardware Mapping ---
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        // --- Drivetrain Config ---
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        // Run drivetrain motors without encoders for responsive control
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- Launcher Config ---
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        // --- Feeders ---
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        telemetry.addData("Status", "Enhanced TeleOp Initialized (v2.1)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Gamepad Inputs ---
        double driveForward = -gamepad1.left_stick_y;
        double driveStrafe = gamepad1.left_stick_x;
        double driveRotate = gamepad1.right_stick_x;
        double launcherTrigger = gamepad1.right_trigger;
        double slowFactor = 1.0 - (0.7 * gamepad1.left_trigger);
        // (0 trigger = full speed, full trigger = 30% speed)
        boolean shotRequested = gamepad1.right_bumper;

        // --- Drive ---
        vectorMecanumDrive(driveStrafe * slowFactor, driveForward * slowFactor, driveRotate * slowFactor);

        // --- Launcher Speed Control with Battery Compensation ---
        double rawTargetVelocity;
        if (launcherTrigger > 0.8) {
            rawTargetVelocity = HIGH_VELOCITY;
        } else if (launcherTrigger > 0.1) {
            rawTargetVelocity = LOW_VELOCITY;
        } else {
            rawTargetVelocity = 0;
        }

        double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double compensatedVelocity = rawTargetVelocity * (NOMINAL_VOLTAGE / currentVoltage);
        currentTargetVelocity = rawTargetVelocity == 0 ? 0 : compensatedVelocity;
        launcher.setVelocity(currentTargetVelocity);

        // --- Rumble Feedback ---
        double currentVelocity = launcher.getVelocity();
        boolean isAtSpeedNow = (currentTargetVelocity > 0) &&
                (Math.abs(currentVelocity - currentTargetVelocity) < VELOCITY_TOLERANCE);

        if (isAtSpeedNow && !flywheelAtSpeed) {
            try {
                gamepad1.rumble(250);
            } catch (Exception ignored) {}
        }
        flywheelAtSpeed = isAtSpeedNow;

        // --- Launcher State Machine ---
        runLaunchLogic(shotRequested);

        // --- Telemetry ---
        updateTelemetry(currentVoltage);
    }

    private void vectorMecanumDrive(double Vx, double Vy, double omega) {
        double lfPower = Vy + Vx + omega * wheelY;
        double rfPower = Vy - Vx - omega * wheelY;
        double lbPower = Vy - Vx + omega * wheelY;
        double rbPower = Vy + Vx - omega * wheelY;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                Math.max(Math.abs(lbPower), Math.abs(rbPower))
        ));

        leftFrontDrive.setPower(clip(lfPower / max));
        rightFrontDrive.setPower(clip(rfPower / max));
        leftBackDrive.setPower(clip(lbPower / max));
        rightBackDrive.setPower(clip(rbPower / max));
    }

    private double clip(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }

    private void runLaunchLogic(boolean shotIsRequested) {
        switch (launchState) {
            case IDLE:
                if (shotIsRequested && currentTargetVelocity > 0) {
                    launchState = LaunchState.WAITING_FOR_SPEED;
                }
                break;

            case WAITING_FOR_SPEED:
                if (flywheelAtSpeed) {
                    feederTimer.reset();
                    launchState = LaunchState.FEEDING;
                }
                if (!shotIsRequested || currentTargetVelocity == 0) {
                    launchState = LaunchState.IDLE;
                }
                break;

            case FEEDING:
                leftFeeder.setPower(FEEDER_POWER);
                rightFeeder.setPower(FEEDER_POWER);

                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    private void updateTelemetry(double voltage) {
        telemetry.addData("--- Drive ---", "Clipped | Slow Mode (LT)");
        telemetry.addData("Voltage", "%.2f V", voltage);

        telemetry.addData("\n--- Launcher ---", "");
        telemetry.addData("State", launchState);
        telemetry.addData("Target Velocity", "%.1f", currentTargetVelocity);
        telemetry.addData("Actual Velocity", "%.1f", launcher.getVelocity());
        telemetry.addData("Flywheel Ready?", flywheelAtSpeed);

        telemetry.update();
    }

    @Override
    public void stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        launcher.setPower(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }
}
