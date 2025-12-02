/*
 * Combined code incorporating the FTC StarterBot's launcher system
 * and a custom vector-based Mecanum drive with an offset wheel for
 * the 2025-2026 FIRST® Tech Challenge season DECODE™.
 *
 * The drive method uses a vector approach to calculate wheel powers,
 * specifically adjusting for an offset Front Right wheel location.
 * The launcher uses DcMotorEx's velocity control (RUN_USING_ENCODER)
 * managed by a state machine for precise shooting.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name = "VectorOffsetLauncherTeleOp", group = "CombinedBot")
public class VectorOffsetLauncherTeleOp extends OpMode {

    // === LAUNCHER CONSTANTS ===
    final double FEED_TIME_SECONDS = 0.20; // Feeder run duration
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    // === DRIVE CONSTANTS (FR Offset) ===
    // Wheel coordinates relative to robot center (meters)
    private double flX = 0.15, flY = 0.15;      // Front Left
    private double frX = 0.15, frY = -0.15;   // Front Right (offset halfway back)
    private double blX = -0.15, blY = 0.15;    // Back Left
    private double brX = -0.15, brY = -0.15;   // Back Right


    // === HARDWARE DECLARATIONS ===
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    ElapsedTime feederTimer = new ElapsedTime();

    // === LAUNCHER STATE MACHINE ===
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    // === INIT METHOD ===
    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        // --- Hardware Mapping ---
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft"); // Renamed for clarity in this combined file
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        // --- Drive Motor Direction ---
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // --- Motor Settings ---
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        launcher.setZeroPowerBehavior(BRAKE);

        frontLeftDrive.setZeroPowerBehavior(BRAKE);
        frontRightDrive.setZeroPowerBehavior(BRAKE);
        backLeftDrive.setZeroPowerBehavior(BRAKE);
        backRightDrive.setZeroPowerBehavior(BRAKE);

        // --- Feeder Servo Setup ---
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Drive Mode", "Vector Mecanum w/ FR Offset");
    }

    // === LOOP METHOD ===
    @Override
    public void loop() {
        // --- Joystick Inputs ---
        // Vy: forward/back, Vx: strafe, omega: rotation
        double Vy = -gamepad1.left_stick_y;      // Forward/backward
        double Vx = gamepad1.left_stick_x * 1.1; // Strafe (scaled slightly)
        double omega = gamepad1.right_stick_x;   // Rotation

        // --- Drive Robot ---
        driveMecanumVectorOffset(Vx, Vy, omega);

        // --- Manual Launcher Control (Y to spin up, B to stop) ---
        if (gamepad1.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) {
            launcher.setVelocity(STOP_SPEED);
        }

        // --- Launcher State Machine (R-Bumper to queue shot) ---
        launch(gamepad1.rightBumperWasPressed());

        // --- Telemetry ---
        telemetry.addLine("=== Drive Input ===");
        telemetry.addData("Vx (Strafe)", "%.2f", Vx);
        telemetry.addData("Vy (Forward)", "%.2f", Vy);
        telemetry.addData("Omega (Rotation)", "%.2f", omega);
        telemetry.addLine("=== Launcher Status ===");
        telemetry.addData("State", launchState);
        telemetry.addData("Motor Speed (Ticks/s)", launcher.getVelocity());

        telemetry.update();
    }

    // === DRIVE FUNCTION (Vector with FR Offset) ===
    private void driveMecanumVectorOffset(double Vx, double Vy, double omega) {
        // Calculate powers using the vector-based inverse kinematics for a custom wheel placement
        // The rotation component uses the wheel's Y-coordinate relative to the center.

        // Front Left: +Vx, +Vy, Rotation proportional to -flY
        double fl = Vy + Vx + omega * (-flY);
        // Front Right: -Vx, +Vy, Rotation proportional to -frY (frY is smaller due to offset)
        double fr = Vy - Vx + omega * (-frY);
        // Back Left: -Vx, +Vy, Rotation proportional to -blY
        double bl = Vy - Vx + omega * (-blY);
        // Back Right: +Vx, +Vy, Rotation proportional to -brY
        double br = Vy + Vx + omega * (-brY);

        // Normalize powers (scale down if any power exceeds 1.0)
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        // Apply normalized powers to each motor
        frontLeftDrive.setPower(fl / max);
        frontRightDrive.setPower(fr / max);
        backLeftDrive.setPower(bl / max);
        backRightDrive.setPower(br / max);
    }

    // === LAUNCHER FUNCTION (State Machine) ===
    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                // Start or maintain target velocity
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                // Transition to LAUNCH once the minimum speed is reached
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                // Start the feeder
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                // Wait for the feed time to elapse
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }

    // === STOP METHOD ===
    @Override
    public void stop() {
        // Ensure all motors are explicitly stopped or braked on OpMode end
    }
}
