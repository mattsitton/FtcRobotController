package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Launcher_Only_TeleOp", group = "Launcher")
public class LauncherOnlyTeleOp extends OpMode {

    // --- Hardware Declarations ---
    private DcMotorEx launcherMotor;
    private CRServo leftFeeder, rightFeeder;
    private VoltageSensor batteryVoltageSensor;

    // --- Timed Launch Variable ---
    private final ElapsedTime feederTimer = new ElapsedTime();
    private boolean launchInProgress = false;

    // --- Launcher Constants ---
    private final double FEED_TIME_SECONDS = 0.20;
    private final double FULL_FEED_POWER = 1.0;
    private final int RPM_TOLERANCE = 100;
    private final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(350, 0, 1, 12);

    // RPM targets for D-pad control (Customize these values)
    private final int RPM_TARGET_UP = 4;
    private final int RPM_TARGET_RIGHT = 3;
    private final int RPM_TARGET_DOWN = 2;
    private final int RPM_TARGET_LEFT = 1;

    // --- State Variables for Telemetry/Logic ---
    private boolean flywheelAtSpeed = false;
    private int currentTargetRPM = 0;
    private double currentFlywheelRPM = 0.0;
    private double currentManualPower = 0.0;


    @Override
    public void init() {
        // --- Hardware Mapping ---
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // --- Motor & Servo Configuration ---
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        telemetry.addLine("Launcher Only TeleOp Initialized.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Read Gamepad Inputs ---
        double manualFlywheelPower = gamepad1.left_trigger;
        currentManualPower = manualFlywheelPower; // Store for telemetry

        boolean pressX = gamepad1.x;

        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadRight = gamepad1.dpad_right;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;

        // --- Flywheel Control ---
        handleFlywheel(dpadUp, dpadRight, dpadDown, dpadLeft, manualFlywheelPower);

        // --- Timed Feeder Control ---
        handleTimedFeeder(pressX);

        // --- Telemetry ---
        updateTelemetry();
    }


    // --- Utility Methods (Required for RPM Conversion) ---
    private double rpmToTicksPerSecond(double rpm) { return (rpm / 60.0) * 28.0; }
    private double ticksPerSecondToRPM(double ticksPerSec) { return (ticksPerSec / 28.0) * 60.0; }


    // --- Feeder Control Method (X Button) ---
    private void handleTimedFeeder(boolean xPressed) {
        if (launchInProgress) {
            // STEP 2: If a launch is in progress, check the timer
            if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);
                launchInProgress = false; // Stop the launch
            }
        } else if (xPressed) { // Unconditional launch on X press
            // STEP 1: If X is pressed, START the launch
            feederTimer.reset();
            leftFeeder.setPower(FULL_FEED_POWER);
            rightFeeder.setPower(FULL_FEED_POWER);
            launchInProgress = true;
        }
    }

    // --- Flywheel Speed Control Method (D-pad / Left Trigger) ---
    private void handleFlywheel(boolean dpadUp, boolean dpadRight, boolean dpadDown, boolean dpadLeft, double manualPower) {

        if (manualPower > 0.1) {
            // MANUAL OVERRIDE (Left Trigger is pressed): Set power directly
            currentTargetRPM = 0; // Clear RPM target
            launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcherMotor.setPower(1);
            currentFlywheelRPM = 0.0; // RPM is not guaranteed/accurate in RUN_WITHOUT_ENCODER
            flywheelAtSpeed = false;
            return;
        }

        // RETURN TO D-PAD RPM CONTROL
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (dpadUp) currentTargetRPM = RPM_TARGET_UP;
        else if (dpadRight) currentTargetRPM = RPM_TARGET_RIGHT;
        else if (dpadDown) currentTargetRPM = RPM_TARGET_DOWN;
        else if (dpadLeft) currentTargetRPM = RPM_TARGET_LEFT;
        else currentTargetRPM = 0; // Turn off the flywheel

        launcherMotor.setVelocity(rpmToTicksPerSecond(currentTargetRPM));

        double currentRPM = ticksPerSecondToRPM(launcherMotor.getVelocity());
        currentFlywheelRPM = currentRPM;

        boolean atSpeedNow = (currentTargetRPM > 0) && (Math.abs(currentRPM - currentTargetRPM) <= RPM_TOLERANCE);
        flywheelAtSpeed = atSpeedNow;
    }


    // --- Telemetry Method ---
    private void updateTelemetry() {
        String launcherStatus = currentTargetRPM > 0 ? "Target: " + currentTargetRPM + " RPM" :
                currentManualPower > 0.1 ? "Manual Power: " + String.format("%.2f", currentManualPower) : "OFF";

        telemetry.addData("Battery", "%.2f V", batteryVoltageSensor.getVoltage());
        telemetry.addLine("-----");
        telemetry.addData("Flywheel Target", launcherStatus);
        telemetry.addData("D-Pad Map", "U:%d R:%d D:%d L:%d", RPM_TARGET_UP, RPM_TARGET_RIGHT, RPM_TARGET_DOWN, RPM_TARGET_LEFT);
        telemetry.addData("Launcher Current", "%.0f RPM", currentFlywheelRPM);
        telemetry.addData("Flywheel Ready (PID)", flywheelAtSpeed);
        telemetry.addData("Feeder Control", "Press X for Timed Feed (%.2fs)", FEED_TIME_SECONDS);
        telemetry.addData("Manual Speed", "Left Trigger");

        telemetry.update();
    }

    @Override
    public void stop() {
        launcherMotor.setPower(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }
}