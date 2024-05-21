package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="swervetest", group="TeleOp")
public class SwerveModuleTest extends OpMode {

    DcMotor top;
    DcMotor bottom;

    public void init() {
        top = hardwareMap.get(DcMotor.class, "top");
        bottom = hardwareMap.get(DcMotor.class, "bottom");

        telemetry.addData("init", "completed");
        telemetry.update();

    }

    @Override
    public void loop() {
        if(gamepad1.y){bottom.setPower(.5); telemetry.addData("Motor: ", "bott up");}
        else if(gamepad1.a){bottom.setPower(-.5); telemetry.addData("Motor: ", "bott down");}
        if(gamepad1.dpad_down){top.setPower(-.5); telemetry.addData("Motor: ", "top up");}
        else if(gamepad1.dpad_up){top.setPower(.5); telemetry.addData("Motor: ", "top dow");}
        else if (!gamepad1.a && !gamepad1.b){
            telemetry.addData("Motor: ", "guess what?");
            top.setPower(0);
            bottom.setPower(0);
        }
        telemetry.update();
    }


}
