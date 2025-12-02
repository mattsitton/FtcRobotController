package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range; // <--- ADDED IMPORT

/**
 * MasterLauncherV3_4 Subsystem (Modular Code)
 * Manages the flywheel motor (PID velocity control) and the timed feeder mechanism.
 * Requires: DcMotorEx, 2 CRServos, VoltageSensor (mapped in the constructor).
 */
public class MasterLauncherV3_4 {

    // --- Hardware ---
    private DcMotorEx launcherMotor;
    private CRServo leftFeeder, rightFeeder;
    private VoltageSensor batteryVoltageSensor;

    // --- Encoder Constant ---
    // GoBilda 5203 (19.1:1) output shaft encoder ticks per revolution (534.8 PPR)
    private final double FLWHEEL_TICKS_PER_REV = 534.8;

    // --- Constants ---
    public final double FEED_TIME_SECONDS = 0.20;
    private final double FULL_FEED_POWER = 1.0;
    private final int RPM_TOLERANCE = 100;
    private final double NOMINAL_VOLTAGE = 12.0;

    // PIDF Coefficients (P=50 for stable PID control, F=12 for power baseline)
    public final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(50, 0, 0, 12);

    // Realistic RPM targets (Public access for OpMode D-pad mapping)
    public final int RPM_TARGET_UP = 300;
    public final int RPM_TARGET_RIGHT = 280;
    public final int RPM_TARGET_DOWN = 260;
    public final int RPM_TARGET_LEFT = 240;

    // --- State Variables ---
    private final ElapsedTime feederTimer = new ElapsedTime();
    private boolean launchInProgress = false;
    private int currentTargetRPM = 0;
    private double currentFlywheelRPM = 0.0;
    private boolean flywheelAtSpeed = false;


    /**
     * Initialize the launcher subsystem and configure hardware.
     * @param hardwareMap The OpMode's hardwareMap object.
     * @param batterySensor The robot's primary voltage sensor.
     */
    public MasterLauncherV3_4(HardwareMap hardwareMap, VoltageSensor batterySensor) {
        // --- Hardware Mapping ---
        // [CUSTOMIZE] Ensure these names match your configuration:
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        this.batteryVoltageSensor = batterySensor;

        // --- Configuration ---
        launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launcherMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
        launcherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }

    // --- Public Control and Update Methods ---

    /**
     * Handles D-pad RPM selection and manual power override.
     * This method must be called every loop.
     * @param dpadRpmTarget The desired RPM target (0 for off).
     * @param manualPower Left Trigger input (0.0 to 1.0).
     */
    public void updateFlywheelControl(int dpadRpmTarget, double manualPower) {

        double voltageComp = NOMINAL_VOLTAGE / batteryVoltageSensor.getVoltage();

        if (manualPower > 0.1) {
            // MANUAL OVERRIDE (Left Trigger)
            currentTargetRPM = 0;
            launcherMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            // Scale input (0.8 pull = 1.0 output)
            double scaledInput = Range.clip(manualPower / 0.8, 0.0, 1.0);

            double compensatedPower = Range.clip(scaledInput * voltageComp, 0.0, 1.0);
            launcherMotor.setPower(compensatedPower);
            currentFlywheelRPM = 0.0;
            flywheelAtSpeed = false;
            return;
        }

        // D-PAD RPM CONTROL
        launcherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        currentTargetRPM = dpadRpmTarget;

        launcherMotor.setVelocity(rpmToTicksPerSecond(currentTargetRPM));

        // --- PID TUNING INSTRUCTIONS ---
        // P-Gain is the proportional response. F-Gain is the base power.
        // F-Gain (12) should remain fixed as it matches the motor's theoretical constant.
        // P-Gain (50) controls stability and response time.
        // 1. If motor STILL OSCILLATES (jerks): REDUCE the P-gain in LAUNCHER_PIDF (e.g., from 50 to 10).
        //    The P-gain must be very small for stable velocity control.
        // 2. If motor is SLOW to reach speed: Slightly INCREASE the P-gain (e.g., from 50 to 60) until stable/fastest response is achieved.

        double currentRPM = ticksPerSecondToRPM(launcherMotor.getVelocity());
        currentFlywheelRPM = currentRPM;

        // Determine if flywheel is at speed
        flywheelAtSpeed = (currentTargetRPM > 0) && (Math.abs(currentRPM - currentTargetRPM) <= RPM_TOLERANCE);
    }

    /**
     * Checks if a timed launch is in progress and executes the feeder logic.
     * @param xPressed X button state.
     */
    public void updateFeederControl(boolean xPressed) {
        if (launchInProgress) {
            // Stop the feeder after the set time
            if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);
                launchInProgress = false;
            }
        } else if (xPressed) {
            // Start a new timed launch (unconditional launch on X press)
            feederTimer.reset();
            leftFeeder.setPower(FULL_FEED_POWER);
            rightFeeder.setPower(FULL_FEED_POWER);
            launchInProgress = true;
        }
    }

    /**
     * Safety method to turn off all motors and feeders on OpMode stop.
     */
    public void stop() {
        launcherMotor.setPower(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }

    // --- Public Getters for Telemetry ---
    public boolean isFlywheelAtSpeed() { return flywheelAtSpeed; }
    public double getCurrentRPM() { return currentFlywheelRPM; }
    public int getTargetRPM() { return currentTargetRPM; }
    public boolean isLaunchInProgress() { return launchInProgress; }

    // --- Conversion Utilities ---
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * FLWHEEL_TICKS_PER_REV;
    }

    private double ticksPerSecondToRPM(double ticksPerSec) {
        return (ticksPerSec / FLWHEEL_TICKS_PER_REV) * 60.0;
    }
}