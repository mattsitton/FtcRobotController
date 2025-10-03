package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Vector Mecanum FR Offset TeleOp + Telemetry", group="TeleOp")
public class TeleOpVectorMecanumFROffsetTelemetry extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Wheel coordinates from robot center
    private double flX = 0.15, flY = 0.15;
    private double frX = 0.15, frY = -0.075; // Front-right offset halfway back
    private double blX = -0.15, blY = 0.15;
    private double brX = -0.15, brY = -0.15;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Vector FR Offset Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        double Vy = -gamepad1.left_stick_y; // Forward/back
        double Vx = gamepad1.left_stick_x * 1.1; // Strafe
        double omega = gamepad1.right_stick_x; // Rotation

        driveMecanumVectorOffset(Vx, Vy, omega);

        telemetry.addData("Vx (Strafe)", "%.2f", Vx);
        telemetry.addData("Vy (Forward)", "%.2f", Vy);
        telemetry.addData("Omega (Rotation)", "%.2f", omega);
        telemetry.addData("FL Power", "%.2f", frontLeft.getPower());
        telemetry.addData("FR Power", "%.2f", frontRight.getPower());
        telemetry.addData("BL Power", "%.2f", backLeft.getPower());
        telemetry.addData("BR Power", "%.2f", backRight.getPower());
        telemetry.update();
    }

    private void driveMecanumVectorOffset(double Vx, double Vy, double omega) {
        // FR wheel rotation scaled smaller due to offset
        double fl = Vy + Vx + omega * (-flY);
        double fr = Vy - Vx + omega * (-frY); // scaled for FR offset
        double bl = Vy - Vx + omega * (-blY);
        double br = Vy + Vx + omega * (-brY);

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }
}
