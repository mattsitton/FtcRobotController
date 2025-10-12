package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * AutonomousForward
 *
 * This OpMode executes a simple one-second movement forward for a Mecanum drive robot.
 * It is a non-encoder based autonomous routine, relying on time (ElapsedTime) for distance control.
 *
 * Motor names and directions match the StarterBotTeleopMecanums configuration:
 * - Motors: left_front_drive, right_front_drive, left_back_drive, right_back_drive
 * - Left side motors are reversed for driving straight.
 */

@Autonomous(name = "Autonomous Forward 1 Second", group = "Autonomous")
public class AutonomousForward extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Constant for the power level and duration
    private final double DRIVE_POWER = 0.5;
    private final double DRIVE_TIME_SECONDS = 1.0;

    @Override
    public void runOpMode() {
        // 1. Initialize Drive Motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        // 2. Set Motor Directions (Matching TeleOp logic)
        // Left side motors are reversed to drive straight forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // 3. Set Motor RunMode and ZeroPowerBehavior (for consistency)
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized and Ready to Run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Check if the OpMode was started and not stopped
        if (opModeIsActive()) {

            // Stage 1: Move Forward for 1 Second
            telemetry.addData("Stage", "1. Driving Forward for %.2f seconds", DRIVE_TIME_SECONDS);
            telemetry.update();

            // Reset the runtime clock
            runtime.reset();

            // Set all motors to the desired power for forward movement
            // Forward movement for a mecanum drive is simply setting all powers to the same value
            setDrivePower(DRIVE_POWER, DRIVE_POWER, DRIVE_POWER, DRIVE_POWER);

            // Keep looping until the elapsed time exceeds the target duration
            while (opModeIsActive() && runtime.seconds() < DRIVE_TIME_SECONDS) {
                // Continue to drive until time is up.
                idle(); // Release the processor for other background tasks
            }

            // Stage 2: Stop
            telemetry.addData("Stage", "2. Stopping Robot");
            telemetry.update();

            // Stop all drive motors
            setDrivePower(0.0, 0.0, 0.0, 0.0);

            // Wait briefly before ending (Optional)
            sleep(500);
        }
    }

    /**
     * Helper method to set motor powers easily.
     * @param lfPower Left Front Power
     * @param rfPower Right Front Power
     * @param lbPower Left Back Power
     * @param rbPower Right Back Power
     */
    private void setDrivePower(double lfPower, double rfPower, double lbPower, double rbPower) {
        leftFrontDrive.setPower(lfPower);
        rightFrontDrive.setPower(rfPower);
        leftBackDrive.setPower(lbPower);
        rightBackDrive.setPower(rbPower);
    }
}