package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libs.Movement;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="liftMover", group="TeleOp")
public class liftMover extends OpMode {


    double[] motorPower = {0, 0, 0, 0};
    DcMotor climberLeft;
    DcMotor climberRight;

    public void init() {
        climberLeft = hardwareMap.get(DcMotor.class, "leftClimber");
        climberRight = hardwareMap.get(DcMotor.class, "rightClimber");


        telemetry.addData("init", "completed");
        telemetry.update();
    }

    @Override
    public void loop()
    {
    climberLeft.setPower(gamepad1.left_stick_y);
    climberRight.setPower(gamepad1.right_stick_y);
    }

}
