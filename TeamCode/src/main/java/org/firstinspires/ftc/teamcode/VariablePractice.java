package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class VariablePractice extends OpMode {
    @Override
    public void init() {
        int teamNumber =23014;
        int motorAngle =66;

        double motorSpeed = 0.75;
        boolean clawClosed = true;
        String TeamName = "Connally Robotics";

        telemetry.addData("Team Number",teamNumber);
        telemetry.addData("motorAngle",motorAngle);
        telemetry.addData("motor Speed",motorSpeed);
        telemetry.addData("claw closed",clawClosed);
        telemetry.addData("Name", TeamName);

    }

    @Override
    public void loop() {
/*
1.change stringvariable name to team name
2. create int called "motorAngle" st or angle between 0-180
 */
    }
}
