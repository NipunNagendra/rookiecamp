package org.firstinspires.ftc.teamcode.TeleOp;

import static com.sun.tools.doclint.Entity.pi;

import android.util.Pair;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.libs.Movement;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MainTeleOpField", group="TeleOp")
public class MainTeleOpField extends OpMode {
    Movement move;
    //RevColorSensorV3 cs;
    //DistanceSensor ds;
    String moveType = "robot";
    boolean outtakeServoStatus = false;
    double[] motorPower = {0, 0, 0, 0};
    double multiplier = 1;
    Pair<float[], String> hsv_color;

    IMU imu;

    public static double heading;

    public static double targetAngle;

    public static String lockStatus;
    public void init() {
        //manip = new Manipulators(hardwareMap);
        //manip.droneServo.setPosition(0.5);
        move = new Movement(hardwareMap);
        //cs = hardwareMap.get(RevColorSensorV3.class, "cs");
        //ds = hardwareMap.get(DistanceSensor.class, "ds");
        gamepad1.setLedColor(0, 0, 256, 100000);
        telemetry.addData("init", "completed");
        telemetry.update();
        // imu for field oriented drive
        imu = hardwareMap.get(IMU.class, "imu");
        //TODO: Change for acc roibot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(new Orientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZXY,
                AngleUnit.DEGREES,
                -90,
                90,
                -33,
                0
        )
        ));
        imu.initialize(parameters);
        imu.resetYaw();
        lockStatus = "unlocked";

    }

    @Override
    public void loop()
    {


        double leftY;
        double leftX;
        double rightX;


        if (move.isPressed("options1",gamepad1.options)) {
            imu.resetYaw();
        }



        if (gamepad1.right_bumper) {multiplier = 0.5;}
        else if (gamepad1.left_bumper) {multiplier = 0.25;}
        else{multiplier=1;}

        heading = move.drive.getExternalHeading();

        if (gamepad1.right_stick_x > 0.1) {
            targetAngle = 0;
        }
        else if(gamepad1.x){
            targetAngle=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) +(-Math.PI/2);
        }
        else if(gamepad1.y){
            targetAngle=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) +(0);
        }
        else if(gamepad1.a){
            targetAngle=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) +(-Math.PI);
        }
        else if(gamepad1.b){
            targetAngle=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) +(Math.PI/2);
        }


        if (Math.abs(gamepad1.left_stick_y)  > 0.1 ||
                Math.abs(gamepad1.left_stick_x)  > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1) {

            leftY = gamepad1.left_stick_y * multiplier;
            leftX = (gamepad1.left_stick_x * multiplier);
            rightX = (gamepad1.right_stick_x * multiplier);

            move.fieldDrive(leftX, leftY, rightX, heading, targetAngle);
        }
        else {
            move.fieldDrive(0,0, 0, heading, targetAngle);
        }

        telemetry.addData("imu:",  heading);
        telemetry.update();
//
//        // driver 2 uses right bumper to toggle the outtake gate
//        if (move.isPressed("rightBumper2", gamepad2.right_bumper)) {
//            manip.gateToggle();
//        }
//        if (move.isPressed("leftBumper2", gamepad2.left_bumper)) {
//            manip.gateToggle1();
//        }

        if (gamepad1.y) {
            
        }


//        if (move.isPressed("rightBumper2", gamepad2.right_bumper)) {
//            manip.gateToggle(outtakeServoStatus);
//            if (outtakeServoStatus == false){
//                outtakeServoStatus = true;
//            } else if(outtakeServoStatus){
//                outtakeServoStatus = false;
//            }
//        }
        // uses dpad controls up and down to control the climber/hanger
//        if(gamepad2.dpad_up){
//            manip.climberLiftPower(.5);
//        } else if(gamepad2.dpad_down){
//            manip.climberLiftPower(-.5);
//        } else{
//            manip.climberLiftPower(0);
//        }
//
//        // When receiving power from gamepad2 that is greater than a certain threshold
//        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
//            // it will move the lift to the certain power that the right joystick set
//            manip.setOuttakeLiftPower(gamepad2.right_stick_y);
//        }
//        else {
//            //
//            manip.setOuttakeLiftPower(0);
//        }
//
//        //uses right and left triggers to control lift
//        if (gamepad2.right_trigger > 0.1){
//            manip.setIntakePower(gamepad2.right_trigger);
//        } else if (gamepad2.left_trigger > 0.1){
//            //left trigger sets intake power in reverse
//            manip.setIntakePower(-1.0 * gamepad2.left_trigger);
//        } else {
//            manip.setIntakePower(0);
//        }
//
//        if (Math.abs(gamepad2.left_stick_y) > 0.1){
//            manip.intakeLeftServo.setPower(gamepad2.left_stick_y);
//            manip.intakeRightServo.setPower(gamepad2.left_stick_y);
//        }
//        else{
//            manip.intakeLeftServo.setPower(0);
//            manip.intakeRightServo.setPower(0);
//        }
//    }

}}
