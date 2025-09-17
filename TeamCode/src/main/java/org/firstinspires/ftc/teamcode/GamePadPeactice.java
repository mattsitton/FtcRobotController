package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp
public class GamePadPeactice extends OpMode {

    @Override
    public void init() {
        //runs 50x* a second
    }

    @Override
    public void loop() {
        double speedForward = -gamepad1.left_stick_y/2.0;
        double deffXJoysticks = gamepad1.left_stick_x-gamepad1.right_stick_x;
        double sumTriggers = gamepad1.right_trigger+gamepad1.left_trigger;

        telemetry.addData("left y",gamepad1.left_stick_x);
        telemetry.addData("left y",speedForward);
        telemetry.addData("A button",gamepad1.a);
        telemetry.addData("right x",gamepad1.right_stick_x);
        telemetry.addData("right y",gamepad1.right_stick_y);
        telemetry.addData("diffence of x ",deffXJoysticks);
        telemetry.addData("sum of trigger",sumTriggers);


    }
}
