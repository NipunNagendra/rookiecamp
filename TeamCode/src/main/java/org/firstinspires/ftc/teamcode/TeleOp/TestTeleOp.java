package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.libs.Movement;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TestTeleOp", group="TeleOp")
public class TestTeleOp extends OpMode {
    Movement move;
    RevColorSensorV3 cs;
    DistanceSensor ds;
    String moveType = "robot";

    double[] motorPower = {0, 0, 0, 0};
    double multiplier;
    public void init() {
        move = new Movement(hardwareMap);
        cs = hardwareMap.get(RevColorSensorV3.class, "cs");
        ds = hardwareMap.get(DistanceSensor.class, "ds");
        telemetry.addData("init", "completed");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        //imu for field oriented drive
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));

        double leftY;
        double leftX;
        double rightX;

        int red = cs.red();
        int green = cs.green();
        int blue = cs.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(red,green,blue,hsv);
        float hue = hsv[0];
        String color;

        if(hue>=190 && hue<=300){
            color="purple";
        }
        else if(hue>=105 && hue<=150 ){
            color="green";
        }
        else if(hue>=16 && hue<=100){
            color="yellow";
        }
        else{
            color="weird color";
        }

        telemetry.addData("Red: ",cs.red());
        telemetry.addData("Green: ",cs.green());
        telemetry.addData("Blue: ",cs.blue());
        telemetry.addData("Hue", hue);
        telemetry.addLine(color);
        telemetry.addData("Distance(Cm)", ds.getDistance(DistanceUnit.CM));
        telemetry.addLine(moveType);
        telemetry.update();

        if(ds.getDistance(DistanceUnit.CM)>=35 && ds.getDistance(DistanceUnit.CM)<=45){
            gamepad1.rumble(1,1,100);
        }

        if (gamepad1.options) {
            imu.resetYaw();
        }

        if (move.isPressed("share", gamepad1.share)) {
            if (moveType == "robot") {
                moveType = "field";
            }
            else if(moveType == "field"){
                moveType = "robot";
            }
        }
        if (moveType == "robot") {
            gamepad1.setLedColor(0,0,256,1000);
        }
        else if (moveType == "field"){
            gamepad1.setLedColor(256,0,0,1000);
        }


        if (move.isColor("yellow", (color.equals("yellow")))){
            gamepad2.setLedColor(255,191,0,100000);}
        else if (move.isColor("purple", (color.equals("purple")))){
            gamepad2.setLedColor(179,0,255,100000);
        }
        else if (move.isColor("green", (color.equals("green")))){
            gamepad2.setLedColor(0,256,0,100000);
        }




        if (Math.abs(gamepad1.left_stick_y)  > 0.1 ||
                Math.abs(gamepad1.left_stick_x)  > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1) {

            leftY = gamepad1.left_stick_y;
            leftX = -1*(gamepad1.left_stick_x);
            rightX = -1*(gamepad1.right_stick_x);

            if(moveType=="robot") {
                motorPower = move.holonomicDrive(leftX, leftY, rightX);
            }
            else if(moveType=="field"){
                motorPower = move.fieldDrive(leftX,leftY, rightX, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            }
        }
        else {
            if(moveType=="robot") {
                motorPower = move.holonomicDrive(0, 0, 0);
            }
            else if(moveType=="field"){
                motorPower = move.fieldDrive(0,0, 0, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            }
        }
        if (gamepad1.right_bumper){multiplier=0.5;}
        if (gamepad1.left_bumper){multiplier=0.25;}
        move.setPowers(motorPower, multiplier);
    }

}
