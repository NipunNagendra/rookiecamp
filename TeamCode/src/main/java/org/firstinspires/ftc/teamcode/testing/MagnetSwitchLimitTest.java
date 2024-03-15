package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.libs.Movement;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MagneticLimitSwitchTest", group="TeleOp")
public class MagnetSwitchLimitTest extends OpMode {


    double[] motorPower = {0, 0, 0, 0};

    TouchSensor mls;

    DcMotor outtakeLiftMotor;;
    DcMotor climberLeft;
    DcMotor climberRight;

    boolean mlsStatus = false;

    public void init() {
        mls = hardwareMap.get(TouchSensor.class, "touchSensor");
        outtakeLiftMotor = hardwareMap.get(DcMotor.class, "outtakeLiftMotor");

        telemetry.addData("init", "completed");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        if(mls.isPressed() && gamepad1.right_stick_y > 0)
        {
            outtakeLiftMotor.setPower(0);
        }
        else
        {
            outtakeLiftMotor.setPower(-1 * gamepad1.right_stick_y);
        }

        if (mls.isPressed())
        {
            mlsStatus = true;
        }
        else
        {
            mlsStatus = false;
        }

        telemetry.addData("joystick value", gamepad1.right_stick_y);
        telemetry.addData("Magnet Status: ", mlsStatus);
        telemetry.update();
    }

}
