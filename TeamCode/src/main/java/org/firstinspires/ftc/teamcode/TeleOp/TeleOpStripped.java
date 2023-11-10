package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.libs.Movement;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpStripped", group="TeleOp")
public class TeleOpStripped extends OpMode {

    Movement move;
    Manipulators manip;

    double[] motorPower = {0, 0, 0, 0};

    public void init() {
        move = new Movement(hardwareMap);
        manip = new Manipulators(hardwareMap);

        telemetry.addData("init", "completed");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        double leftY = 0;
        double leftX = 0;
        double rightX = 0;


        if (Math.abs(gamepad1.left_stick_y)  > 0.1 ||
                Math.abs(gamepad1.left_stick_x)  > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1) {

            leftY = gamepad1.left_stick_y/1.5;
            leftX = -1*(gamepad1.left_stick_x);
            rightX = (-gamepad1.right_stick_x)/1.5;

            motorPower = move.holonomicDrive(leftX, leftY, rightX);
        }
        else {
            motorPower = move.holonomicDrive(0, 0, 0);
        }

        move.setPowers(motorPower[0], motorPower[1], motorPower[2], motorPower[3]);
        manip.leftClimberMotor.setPower(gamepad2.left_stick_y);
        manip.rightClimberMotor.setPower(gamepad2.right_stick_y);

    }


}
