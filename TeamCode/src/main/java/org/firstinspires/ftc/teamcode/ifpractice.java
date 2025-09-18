package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ifpractice extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
       double motorSpeed = gamepad1.left_stick_y;
      if (!gamepad1.a ){
          motorSpeed *= .5;
      }


       telemetry.addData("Left stick value",motorSpeed);
    }
}
