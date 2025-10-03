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

@TeleOp(name = "StarterBot_Final_MethodTelemetry", group = "StarterBot")
@Disabled
public class StarterBotTeleopMecanums extends OpMode {

    // --- Constants & Tuning Variables ---
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    // Asymmetrical Wheel Coordinates for Vector Drive
    private double flY = 0.15;
    private double frY = -0.075; // Front-right offset halfway back
    private double blY = 0.15;
    private double brY = -0.15;

    // --- Hardware Declarations ---
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;
    private CRServo intakeLeft, intakeRight;

    // --- State Machine & Control Variables ---
    ElapsedTime feederTimer = new ElapsedTime();
    private enum LaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING }
    private LaunchState launchState;
    private boolean intakeOn = false;
    private boolean lastB = false;


    // =========================================================================================
    //                                     OPMODE CORE METHODS
    // =========================================================================================

    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        // Initialize all hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");

        // Set motor directions for the StarterBot chassis
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        // Run all robot functions. The helper methods will now handle
        // staging their own telemetry data.
        handleDrive();
        handleIntake();
        handleFlywheel();
        handleLaunch();

        // The single update call sends all staged data to the Driver Station.
        telemetry.update();
    }

    @Override
    public void stop() {}


    // =========================================================================================
    //                                     HELPER METHODS
    // =========================================================================================

    /**
     * Handles the drivetrain logic and telemetry.
     */
    void handleDrive() {
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);
    }

    /**
     * Handles the intake logic and telemetry.
     */
    void handleIntake() {
        if (gamepad1.b && !lastB) {
            intakeOn = !intakeOn;
        }
        lastB = gamepad1.b;
        intakeLeft.setPower(intakeOn ? 1.0 : 0.0);
        intakeRight.setPower(intakeOn ? 1.0 : 0.0);
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
    }

    /**
     * Handles the manual flywheel controls and telemetry.
     */
    void handleFlywheel() {
        double trigger = gamepad1.left_trigger;
        if (trigger > 0.8) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY); // High Speed
        } else if (trigger > 0.4) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY * 0.7); // Medium Speed
        } else {
            if (launchState == LaunchState.IDLE) {
                launcher.setVelocity(0.0); // Off
            }
        }
        telemetry.addData("Flywheel Target Velocity", "%.1f", launcher.getVelocity());
    }

    /**
     * Handles the auto-launch state machine.
     */
    void handleLaunch() {
        launch(gamepad1.rightBumperWasPressed());
    }

    /**
     * This is the vector-based mecanum drive logic.
     * It uses the physical coordinates of each wheel for accurate turning.
     * @param Vy     The desired forward speed (-1 to 1).
     * @param Vx     The desired strafing speed (-1 to 1).
     * @param omega  The desired turning speed (-1 to 1).
     */
    void mecanumDrive(double Vy, double Vx, double omega) {
        // Calculate power for each wheel using the vector formula
        double fl = Vy + Vx + omega * (-flY);
        double fr = Vy - Vx + omega * (-frY); // Scaled for FR offset
        double bl = Vy - Vx + omega * (-blY);
        double br = Vy + Vx + omega * (-brY);

        // Normalize motor powers
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        // Set power to motors
        leftFrontDrive.setPower(fl / max);
        rightFrontDrive.setPower(fr / max);
        leftBackDrive.setPower(bl / max);
        rightBackDrive.setPower(br / max);

        // Stage telemetry data for the drivetrain
        telemetry.addData("LF Power", "%.2f", leftFrontDrive.getPower());
        telemetry.addData("RF Power", "%.2f", rightFrontDrive.getPower());
        telemetry.addData("LB Power", "%.2f", leftBackDrive.getPower());
        telemetry.addData("RB Power", "%.2f", rightBackDrive.getPower());
        telemetry.addLine(); // Separator for readability
    }

    /**
     * Controls the launcher and feeder state machine.
     * @param shotRequested True if the driver requests a shot.
     */
    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
        // Stage telemetry data for the launch system
        telemetry.addData("Launch State", launchState);
    }
}