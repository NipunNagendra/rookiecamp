package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.libs.Movement;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MotorTest", group="TeleOp")
public class MotorTest extends OpMode {

    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    DcMotor intakeMotor;
    DcMotor outtakeLiftMotor;
    DcMotor leftClimberMotor;
    DcMotor rightClimberMotor;

    public void init() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        outtakeLiftMotor = hardwareMap.get(DcMotor.class, "outtakeLiftMotor");
        leftClimberMotor = hardwareMap.get(DcMotor.class, "leftClimberMotor");
        rightClimberMotor = hardwareMap.get(DcMotor.class, "rightClimberMotor");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);


        telemetry.addData("init", "completed");
        telemetry.update();

    }

    @Override
    public void loop() {
        if(gamepad1.x){FL.setPower(.5); telemetry.addData("Motor: ", "Front Left");}
        else if(gamepad1.y){FR.setPower(.5); telemetry.addData("Motor: ", "Front Right");}
        else if(gamepad1.b){BR.setPower(.5); telemetry.addData("Motor: ", "Back Right");}
        else if(gamepad1.a){BL.setPower(.5); telemetry.addData("Motor: ", "Back Left");}
        else if(gamepad1.dpad_down){rightClimberMotor.setPower(.5); telemetry.addData("Motor: ", "right climber");}
        else if(gamepad1.dpad_left){leftClimberMotor.setPower(.5); telemetry.addData("Motor: ", "left climber");}
        else if(gamepad1.dpad_right){intakeMotor.setPower(.5); telemetry.addData("Motor: ", "intake");}
        else if(gamepad1.dpad_up){outtakeLiftMotor.setPower(.5); telemetry.addData("Motor: ", "outtake");}
        else{
            telemetry.addData("Motor: ", "guess what?");
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            rightClimberMotor.setPower(0);
            leftClimberMotor.setPower(0);
            outtakeLiftMotor.setPower(0);
            intakeMotor.setPower(0);
        }
        telemetry.update();
    }


}
