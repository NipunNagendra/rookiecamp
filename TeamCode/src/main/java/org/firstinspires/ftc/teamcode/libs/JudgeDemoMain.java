package org.firstinspires.ftc.teamcode.libs;

import static com.sun.tools.doclint.Entity.pi;

import android.annotation.SuppressLint;
import android.util.Pair;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.libs.Movement;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="JudgeDemoMain", group="TeleOp")
public class JudgeDemoMain extends OpMode {
    Movement move;

    //RevColorSensorV3 cs;
    DistanceSensor ds;
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
         //manip.droneServo.setPosition(0.5);
        move = new Movement(hardwareMap);
        //cs = hardwareMap.get(RevColorSensorV3.class, "cs");
        ds = hardwareMap.get(DistanceSensor.class, "ds2");
        gamepad1.setLedColor(0, 0, 256, 100000);
        telemetry.addData("init", "completed");
        telemetry.update();
        // imu for field oriented drive
        imu = hardwareMap.get(IMU.class, "imu");
        //TODO: Change for acc roibot

        // season bot
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

        // trollbot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
//        ));

        imu.initialize(parameters);
        imu.resetYaw();
        lockStatus = "unlocked";



    }

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void loop() {


        double leftY;
        double leftX;
        double rightX;


        if (move.isPressed("options1", gamepad1.options)) {
            imu.resetYaw();
        }

        if (gamepad1.right_bumper) {
            multiplier = 0.5;
        } else if (gamepad1.left_bumper) {
            multiplier = 0.25;
        } else {
            multiplier = 1;
        }

        heading = move.drive.getExternalHeading();


        if (gamepad1.right_stick_x > 0.01) {
            targetAngle = 0;
            lockStatus = "unlocked";
        } else if (move.isPressed("x", gamepad1.x)) {
            if (lockStatus == "left") lockStatus = "unlocked";
            else lockStatus = "left";
        } else if (move.isPressed("y", gamepad1.y)) {
            if (lockStatus == "up") lockStatus = "unlocked";
            else lockStatus = "up";
        } else if (move.isPressed("a", gamepad1.a)) {
            if (lockStatus == "down") lockStatus = "unlocked";
            else lockStatus = "down";
        } else if (move.isPressed("b", gamepad1.b)) {
            if (lockStatus == "right") lockStatus = "unlocked";
            else lockStatus = "right";
        }

        if (lockStatus == "unlocked") {
            targetAngle = 0;
            gamepad1.setLedColor(0, 0, 255, 2000);
        } else if (lockStatus == "left") {
            targetAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + (-Math.PI / 2);
            gamepad1.setLedColor(255, 0, 0, 2000);
        }
        if (lockStatus == "down") {
            targetAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + (Math.PI / 2);
            gamepad1.setLedColor(255, 0, 0, 2000);
        } else if (lockStatus == "up") {
            targetAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + (0);
            gamepad1.setLedColor(255, 0, 0, 2000);
        } else if (lockStatus == "down") {
            targetAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            gamepad1.setLedColor(255, 0, 0, 2000);
            if (targetAngle > 0) {
                targetAngle -= Math.PI;
            } else {
                targetAngle += Math.PI;
            }
        } else if (lockStatus == "right")
            targetAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + (Math.PI / 2);

        telemetry.addData("lockStatus: ", lockStatus);
        telemetry.addData("targetAngle: ", targetAngle);

        if (Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1) {

            leftY = gamepad1.left_stick_y * multiplier;
            leftX = (gamepad1.left_stick_x * multiplier);
            rightX = (gamepad1.right_stick_x * multiplier);

            move.fieldDrive(leftX, leftY, rightX, heading, targetAngle);
        } else {
            move.fieldDrive(0, 0, 0, heading, targetAngle);
        }

        telemetry.addData("imu:", heading);

        if (ds.getDistance(DistanceUnit.CM) <= 10) {
            gamepad1.rumble(1, 1, 100);
        }

        // driver 2 uses right bumper to toggle the outtake gate
//        if (move.isPressed("rightBumper2", gamepad2.right_bumper)) {
//            gamepad2.setLedColor(255,255,0, 200);
//            manip.gateToggle();
//        }
//        if (move.isPressed("leftBumper2", gamepad2.left_bumper)) {
//            gamepad2.setLedColor(255,255,0, 200);
//            manip.gateToggle1();
//        }
//
//        // When receiving power from gamepad2 that is greater than a certain threshold
//        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
//            // it will move the lift to the certain power that the right joystick set
//            if(manip.liftTouchSensor.isPressed() && gamepad2.right_stick_y> 0){
//                manip.setOuttakeLiftPower(0);
//            }
//            else{
//                manip.setOuttakeLiftPower(gamepad2.right_stick_y);
//            }
//        }
//        else {
//            //
//            manip.setOuttakeLiftPower(0);
//        }
//
//    }}
    }}