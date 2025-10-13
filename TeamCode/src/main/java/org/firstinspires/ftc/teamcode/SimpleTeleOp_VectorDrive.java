// File: SimpleTeleOp_VectorDrive.java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * A simplified TeleOp for testing a vector-based mecanum drive with a
 * front-right wheel offset.
 *
 * This program contains only the drive logic and detailed telemetry to help
 * verify hardware configuration and driving behavior.
 *
 * Controls (Gamepad 1):
 * - Left Stick: Forward/Backward and Strafing.
 * - Right Stick X: Rotation.
 */
@TeleOp(name = "SimpleTeleOp_VectorDrive", group = "TeleOp_Tests")
public class SimpleTeleOp_VectorDrive extends OpMode {

    // --- Drivetrain ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // --- Mecanum Wheel Geometry (in meters from robot center) ---
    private final double flX = 0.15, flY = 0.15;
    private final double frX = 0.15, frY = -0.15; // Front-right wheel is offset halfway back
    private final double blX = -0.15, blY = 0.15;
    private final double brX = -0.15, brY = -0.15;

    @Override
    public void init() {
        // --- Hardware Mapping ---
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // --- Motor Configuration ---
        // Reverse the left motors because they are mounted opposite to the right ones.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Simple Vector Drive TeleOp Initialized.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Read Gamepad Inputs ---
        double Vy = -gamepad1.left_stick_y;      // Forward/Backward
        double Vx = gamepad1.left_stick_x * 1.1; // Strafe (scaled for more speed)
        double omega = gamepad1.right_stick_x;   // Rotation

        // --- Drive Robot ---
        driveMecanumVectorOffset(Vx, Vy, omega);

        // --- Telemetry ---
        telemetry.addLine("--- Joystick Inputs ---");
        telemetry.addData("Forward (Vy)", "%.2f", Vy);
        telemetry.addData("Strafe (Vx)", "%.2f", Vx);
        telemetry.addData("Rotate (omega)", "%.2f", omega);
        telemetry.addLine("\n--- Motor Powers ---");
        telemetry.addData("Front Left", "%.2f", frontLeft.getPower());
        telemetry.addData("Front Right", "%.2f", frontRight.getPower());
        telemetry.addData("Back Left", "%.2f", backLeft.getPower());
        telemetry.addData("Back Right", "%.2f", backRight.getPower());
        telemetry.update();
    }

    /**
     * Calculates and applies motor powers for vector-based mecanum drive.
     * This version correctly handles non-symmetrical wheel placements.
     * @param Vx     Desired strafe speed (-1 to 1).
     * @param Vy     Desired forward speed (-1 to 1).
     * @param omega  Desired rotational speed (-1 to 1).
     */
    private void driveMecanumVectorOffset(double Vx, double Vy, double omega) {
        // Corrected vector-based formula. Rotational power is scaled by wheel's Y-offset.
        double flPower = Vy + Vx - omega * flY;
        double frPower = Vy - Vx - omega * frY;
        double blPower = Vy - Vx - omega * blY;
        double brPower = Vy + Vx - omega * brY;

        // Normalize all wheel powers if any exceed 1.0 to maintain proportions
        double max = Math.max(1.0, Math.abs(flPower));
        max = Math.max(max, Math.abs(frPower));
        max = Math.max(max, Math.abs(blPower));
        max = Math.max(max, Math.abs(brPower));

        frontLeft.setPower(flPower / max);
        frontRight.setPower(frPower / max);
        backLeft.setPower(blPower / max);
        backRight.setPower(brPower / max);
    }
}