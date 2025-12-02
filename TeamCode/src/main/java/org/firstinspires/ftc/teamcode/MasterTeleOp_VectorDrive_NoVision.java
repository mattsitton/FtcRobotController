package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// Removed PIDFCoefficients import as it's no longer used for control
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name = "MasterTeleOp_NoVision", group = "TeleOp")
public class MasterTeleOp_VectorDrive_NoVision extends OpMode {

    // --- Drivetrain ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // --- Mecanum Wheel Geometry (in meters from robot center) ---
    private final double flX = 0.15, flY = 0.15;
    private final double frX = 0.15, frY = -0.075;
    private final double blX = -0.15, blY = 0.15;
    private final double brX = -0.15, brY = -0.15;

    // --- Launcher & Feeder ---
    private DcMotorEx launcherMotor;
    private CRServo leftFeeder, rightFeeder;
    private final double FEED_TIME_SECONDS = 0.20;
    private final double FULL_FEED_POWER = 1.0;

    // 🔥 CHANGE: Define power levels instead of RPMs
    private final double LOW_POWER = 0.6; // Example low power
    private final double HIGH_POWER = 1.0; // Full power

    // NOTE: RPM_TOLERANCE and LAUNCHER_PIDF are no longer used for control logic
    private final int RPM_TOLERANCE = 100;
    // private final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(300, 0, 0, 10); // Removed

    // --- Enhancements ---
    private VoltageSensor batteryVoltageSensor;
    private final double NOMINAL_VOLTAGE = 12.0;

    // --- State Machines & Timers ---
    private final ElapsedTime feederTimer = new ElapsedTime();
    private enum LaunchState { IDLE, LAUNCHING }
    private LaunchState launchState = LaunchState.IDLE;

    // --- State Variables ---
    private boolean flywheelAtSpeed = false;
    // 🔥 CHANGE: currentTargetRPM becomes currentTargetPower
    private double currentTargetPower = 0.0;
    private double currentRPM = 0.0; // NEW: To hold the measured RPM for telemetry


    @Override
    public void init() {
        // --- Hardware Mapping ---
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // --- Motor & Servo Configuration ---
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // 🔥 CHANGE: Set motor to RUN_WITHOUT_ENCODER for power control
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // REMOVED: Custom PIDF setting is not needed in power mode
        // launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        telemetry.addLine("Master TeleOp (No Vision) Initialized.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Read Gamepad Inputs ---
        double stickForward = -gamepad1.left_stick_y;
        double stickStrafe = gamepad1.left_stick_x;
        double stickRotate = gamepad1.right_stick_x;
        double leftTrigger = gamepad1.left_trigger;
        double rightTrigger = gamepad1.right_trigger;
        boolean pressX = gamepad1.x;

        // --- Flywheel Control ---
        handleFlywheel(rightTrigger);

        // --- Apply Driver Enhancements to manual inputs ---
        double slowFactor = 1.0 - (0.7 * leftTrigger);
        double voltageComp = NOMINAL_VOLTAGE / batteryVoltageSensor.getVoltage();

        double forwardCmd = stickForward * slowFactor * voltageComp;
        double strafeCmd  = stickStrafe  * slowFactor * voltageComp;
        double rotateCmd  = stickRotate  * slowFactor * voltageComp;

        // --- Final Drive Command ---
        driveMecanumVectorOffset(strafeCmd, forwardCmd, rotateCmd);

        // --- Launcher State Machine ---
        // For power mode, assume 'ready' if power is applied and actual RPM is high
        boolean isFlywheelReady = currentTargetPower > 0.5 && currentRPM > 2000;
        handleLauncherState(pressX, isFlywheelReady);

        // --- Telemetry ---
        updateTelemetry();
    }

    private void handleFlywheel(double trigger) {
        // 🔥 CHANGE: Set target power based on the trigger
        if (trigger > 0.8) {
            currentTargetPower = HIGH_POWER;
        } else if (trigger > 0.3) {
            currentTargetPower = LOW_POWER;
        } else {
            currentTargetPower = 0.0;
        }

        // Apply power directly
        launcherMotor.setPower(currentTargetPower);

        // 🔥 NEW: Measure and store the actual RPM using the motor encoder
        currentRPM = ticksPerSecondToRPM(launcherMotor.getVelocity());

        // Update state variable: Assume "at speed" if power is applied and measured RPM is reasonable
        flywheelAtSpeed = (currentTargetPower > 0) && (currentRPM > 2000);

        // Optional: Add rumble logic based on currentRPM if desired, but removed old RPM check
    }

    private void handleLauncherState(boolean xPressed, boolean isFlywheelReady) {
        if (launchState == LaunchState.IDLE && xPressed && isFlywheelReady) {
            feederTimer.reset();
            leftFeeder.setPower(FULL_FEED_POWER);
            rightFeeder.setPower(FULL_FEED_POWER);
            launchState = LaunchState.LAUNCHING;
        } else if (launchState == LaunchState.LAUNCHING && feederTimer.seconds() > FEED_TIME_SECONDS) {
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
            launchState = LaunchState.IDLE;
        }
    }

    private void driveMecanumVectorOffset(double Vx, double Vy, double omega) {
        // ... (Drive logic remains the same) ...
        double flPower = Vy + Vx - omega * flY;
        double frPower = Vy - Vx - omega * frY;
        double blPower = Vy - Vx - omega * blY;
        double brPower = Vy + Vx - omega * brY;

        // Find the maximum absolute power among all wheels
        double max = Math.abs(flPower);
        if (Math.abs(frPower) > max) {
            max = Math.abs(frPower);
        }
        if (Math.abs(blPower) > max) {
            max = Math.abs(blPower);
        }
        if (Math.abs(brPower) > max) {
            max = Math.abs(brPower);
        }

        // If the maximum is greater than 1.0, scale all powers down
        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    private void updateTelemetry() {
        telemetry.addData("Mode", "MANUAL");
        telemetry.addData("Battery", "%.2f V", batteryVoltageSensor.getVoltage());

        // 🔥 CHANGE: Show current power and actual measured RPM
        telemetry.addData("Launcher Power", "%.2f", currentTargetPower);
        telemetry.addData("Launcher Actual", "%.0f RPM", currentRPM);

        telemetry.addData("Flywheel Ready", flywheelAtSpeed);
        telemetry.update();
    }

    // REMOVED: rpmToTicksPerSecond is not needed for control
    // private double rpmToTicksPerSecond(double rpm) { return (rpm / 60.0) * 28.0; }

    // CHANGED: ticksPerSecondToRPM is kept to measure and display speed
    private double ticksPerSecondToRPM(double ticksPerSec) { return (ticksPerSec / 28.0) * 60.0; }

    @Override
    public void stop() {
        launcherMotor.setPower(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }
}
