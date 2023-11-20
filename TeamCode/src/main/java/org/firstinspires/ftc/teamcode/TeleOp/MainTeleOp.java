package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;
import android.util.Pair;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.libs.Movement;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends OpMode {
    Manipulators manip;
    Movement move;
    //RevColorSensorV3 cs;
    //DistanceSensor ds;
    String moveType = "robot";
    boolean outtakeServoStatus = false;
    double[] motorPower = {0, 0, 0, 0};
    double multiplier;
    Pair<float[], String> hsv_color;

    public void init() {
        manip = new Manipulators(hardwareMap);
        move = new Movement(hardwareMap);
        //cs = hardwareMap.get(RevColorSensorV3.class, "cs");
        //ds = hardwareMap.get(DistanceSensor.class, "ds");
        manip = new Manipulators(hardwareMap);
        gamepad1.setLedColor(0, 0, 256, 100000);
        telemetry.addData("init", "completed");
        telemetry.update();
    }

    @Override
    public void loop()
    {

        double leftY;
        double leftX;
        double rightX;

        if (manip.getNormalizedColor().second == "yellow" &&
                manip.isColor(manip.getNormalizedColor().second)){
            gamepad2.setLedColor(255, 191, 0, 100000);
        } else if (manip.getNormalizedColor().second == "purple" &&
                manip.isColor(manip.getNormalizedColor().second) ){
            gamepad2.setLedColor(179, 0, 255, 100000);
        } else if (manip.getNormalizedColor().second == "green" &&
                manip.isColor(manip.getNormalizedColor().second)){
            gamepad2.setLedColor(0, 256, 0, 100000);
        }





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

        move.setPowers(motorPower, multiplier);



        if (gamepad1.right_bumper) {multiplier = 0.5;}
        else if (gamepad1.left_bumper) {multiplier = 0.25;}
        else{multiplier=1;}

        move.setPowers(motorPower, multiplier);

        // driver 2 uses the left bumper to launch the drone in endgame
        if (move.isPressed("droneServo", gamepad2.left_bumper)) manip.droneServo.setPosition(0.4);;

        // driver 2 uses right bumper to toggle the outtake gate
//        if (move.isPressed("rightBumper2", gamepad2.right_bumper)) {
//            manip.gateToggle(outtakeServoStatus);
//            if (outtakeServoStatus == false){
//                outtakeServoStatus = true;
//            } else if(outtakeServoStatus){
//                outtakeServoStatus = false;
//            }
//        }

        // uses dpad controls up and down to control the climber/hanger
        if(gamepad2.dpad_up){
            manip.climberLiftPower(.5);
        } else if(gamepad2.dpad_down){
            manip.climberLiftPower(-.5);
        } else{
            manip.climberLiftPower(0);
        }

        // When receiving power from gamepad2 that is greater than a certain threshold
        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            // it will move the lift to the certain power that the right joystick set
            manip.setOuttakeLiftPower(gamepad2.right_stick_y);
        }
        else {
            //
            manip.setOuttakeLiftPower(0);
        }

        //uses right and left triggers to control INTAKE
        if (gamepad2.right_trigger > 0.1){
            manip.setIntakePower(gamepad2.right_trigger);
        } else if (gamepad2.left_trigger > 0.1){
            //left trigger sets intake power in reverse
            manip.setIntakePower(-1.0 * gamepad2.left_trigger);
        } else {
            manip.setIntakePower(0);
        }
    }

}
