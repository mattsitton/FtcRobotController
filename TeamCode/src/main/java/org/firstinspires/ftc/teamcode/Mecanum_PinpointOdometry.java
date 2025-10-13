package org.firstinspires.ftc.teamcode; // Ensure this is your team's package name

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Pinpoint Odometry Imports
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="Mecanum + Pinpoint Odometry", group="Robot")
public class Mecanum_PinpointOdometry extends LinearOpMode {

    // --- Motor Declarations ---
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    // --- Odometry Sensor Declaration ---
    private GoBildaPinpointDriver pinpoint = null;

    // --- Drive Constants ---
    // Adjust this for teleop speed limit (0.0 to 1.0)
    private final double DRIVE_SPEED_SCALE = 0.8;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        // 1. Initialize Motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Set Motor Directions (Adjust REVERSE/FORWARD based on testing)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // 2. Initialize and Configure Pinpoint
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            configurePinpoint();
        } catch (Exception e) {
            telemetry.addData("ERROR", "Pinpoint not found or failed to initialize.");
        }

        // --- Initialization Telemetry ---
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Pinpoint", pinpoint != null ? "READY" : "MISSING");
        telemetry.update();

        // 3. Wait for Start
        waitForStart();
        runtime.reset();

        // 4. Run Loop
        while (opModeIsActive()) {

            // --- Drive Control ---
            // Strafe (Left Stick X) and Forward (Left Stick Y) are typically inverted from gamepad
            double axial   = -gamepad1.left_stick_y * DRIVE_SPEED_SCALE;
            double lateral =  gamepad1.left_stick_x * DRIVE_SPEED_SCALE;
            double yaw     =  gamepad1.right_stick_x * DRIVE_SPEED_SCALE;

            // Use the corrected drive function
            driveMecanumVector(lateral, axial, yaw);


            // --- Pinpoint Update & Telemetry ---
            if (pinpoint != null) {
                // IMPORTANT: This call triggers the odometry computation.
                pinpoint.update();
                Pose2D pose2D = pinpoint.getPosition();

                // Telemetry (Use gamepad A to reset position)
                if (gamepad1.a) {
                    // Reset position to (0, 0, 0) for a clean start/re-reference
                    pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
                    telemetry.addData("Odometry Reset", "Position (0, 0, 0)");
                }

                telemetry.addData("X (in)", "%.2f", pose2D.getX(DistanceUnit.INCH));
                telemetry.addData("Y (in)", "%.2f", pose2D.getY(DistanceUnit.INCH));
                telemetry.addData("Heading (deg)", "%.2f", pose2D.getHeading(AngleUnit.DEGREES));
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    /**
     * Corrected Mecanum Kinematic function (see Section I).
     * @param Vx    Strafe power (Right is Positive)
     * @param Vy    Forward/Backward power (Forward is Positive)
     * @param omega Rotation power (Counter-Clockwise is Positive)
     */
    private void driveMecanumVector(double Vx, double Vy, double omega) {
        double flPower = Vy + Vx + omega;
        double frPower = Vy - Vx - omega;
        double blPower = Vy - Vx + omega;
        double brPower = Vy + Vx - omega;

        // Normalization (find max and scale down to keep proportions)
        double max = Math.max(Math.abs(flPower), Math.abs(frPower));
        max = Math.max(max, Math.abs(blPower));
        max = Math.max(max, Math.abs(brPower));

        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        frontLeftDrive.setPower(flPower);
        frontRightDrive.setPower(frPower);
        backLeftDrive.setPower(blPower);
        backRightDrive.setPower(brPower);
    }

    /**
     * Initializes and configures the Pinpoint Odometry sensor.
     * YOU MUST CUSTOMIZE THE VALUES HERE BASED ON YOUR ROBOT.
     */
    private void configurePinpoint() {
        // --- STEP 1: SET OFFSETS (Measure these on your robot!) ---
        // X-Offset: Distance sideways from the tracking point (Left +, Right -)
        // Y-Offset: Distance forwards from the tracking point (Forward +, Backward -)
        final double POD_X_OFFSET_MM = -84.0;
        final double POD_Y_OFFSET_MM = -168.0;

        pinpoint.setOffsets(POD_X_OFFSET_MM, POD_Y_OFFSET_MM, DistanceUnit.MM); // Example from sample

        // --- STEP 2: SET ENCODER TYPE ---
        // Use goBILDA_4_BAR_POD for your 32mm wheels.
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // --- STEP 3: SET ENCODER DIRECTIONS (Verify these with a simple manual push test!) ---
        // X-pod (Forward/Backward) should increase when robot moves FORWARD.
        // Y-pod (Strafe) should increase when robot moves LEFT.
        final GoBildaPinpointDriver.EncoderDirection X_POD_DIR = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        final GoBildaPinpointDriver.EncoderDirection Y_POD_DIR = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        pinpoint.setEncoderDirections(X_POD_DIR, Y_POD_DIR);

        // --- STEP 4: CALIBRATE AND RESET ---
        // This resets the position to (0, 0, 0) and performs an IMU recalibration.
        // Robot must be stationary for the recalibration to be effective.
        pinpoint.resetPosAndIMU();
    }
}