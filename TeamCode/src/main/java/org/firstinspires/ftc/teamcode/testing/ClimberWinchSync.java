package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.libs.Movement;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ClimberWinchSync", group="TeleOp")
public class ClimberWinchSync extends OpMode {


    double[] motorPower = {0, 0, 0, 0};
    DcMotor climberLeft;
    DcMotor climberRight;

    CRServo winchLeft;
    CRServo winchRight;

    public static double climberPower = .3;
    public static double winchPower = .5;


    public void init() {
        climberLeft = hardwareMap.get(DcMotor.class, "leftClimberMotor");
        climberRight = hardwareMap.get(DcMotor.class, "rightClimberMotor");

        winchLeft = hardwareMap.get(CRServo.class, "winchLeft");
        winchRight = hardwareMap.get(CRServo.class, "winchRight");
        winchLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                telemetry.addData("init", "completed");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        if (gamepad1.dpad_up){
            climberLeft.setPower(climberPower);
            climberRight.setPower(climberPower);
            winchLeft.setPower(-winchPower);
            winchRight.setPower(winchPower);
        }
        else if (gamepad1.dpad_down){
            climberLeft.setPower(-1 * climberPower);
            climberRight.setPower(-1 * climberPower);
            winchLeft.setPower(winchPower);
            winchRight.setPower(-1 * winchPower);
        } else if (gamepad1.triangle) {
            winchLeft.setPower(-winchPower);
            winchRight.setPower(winchPower);
        }
        else if(gamepad1.x){
            winchLeft.setPower(winchPower);
            winchRight.setPower(-1 * winchPower);
        }
        else{
            climberLeft.setPower(0);
            climberRight.setPower(0);
            winchLeft.setPower(0);
            winchRight.setPower(0);
        }


    }

}
