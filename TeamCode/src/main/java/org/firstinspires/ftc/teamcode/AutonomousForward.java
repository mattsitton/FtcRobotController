package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@Autonomous(name = "Autonomous Forward 3 Second", group = "Autonomous")
public class AutonomousForward extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Constant for the power level and duration
    private final double DRIVE_POWER = 0.7  ;
    private final double DRIVE_TIME_SECONDS = 3.0;
/*
      frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
 */
    @Override
    public void runOpMode() {
        // 1. Initialize Drive Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // 2. Set Motor Directions (Matching TeleOp logic)
        // Left side motors are reversed to drive straight forward.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // 3. Set Motor RunMode and ZeroPowerBehavior (for consistency)
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        frontLeft.setPower(lfPower);
        frontRight.setPower(rfPower);
        backLeft.setPower(lbPower);
        backRight.setPower(rbPower);
    }
}