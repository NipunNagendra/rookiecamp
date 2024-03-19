package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;
import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.libs.Movement;
import org.firstinspires.ftc.teamcode.libs.SensorLibrary;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MainTeleOpFieldExpExp", group="TeleOp")
public class MainTeleOpFieldExpExp extends OpMode {
    Movement move;

    Manipulators manip;

    DistanceSensor ds;

    public static double threshold =15;
    SensorLibrary sensor;
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

    public static boolean invisStatus = false;
    public static int outtakeEncoderTicks = 2000;

    public static double climberPower = .3;
    public static double winchPower = .5;
    boolean augmentedSwerve;

    public void init() {
        manip = new Manipulators(hardwareMap);
//        manip.droneServo.setPower(0);
        move = new Movement(hardwareMap);
//        sensor = new SensorLibrary(hardwareMap);
        //cs = hardwareMap.get(RevColorSensorV3.class, "cs");
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
    public void loop()
    {


        double leftY;
        double leftX;
        double rightX;



        if (move.isPressed("options1",gamepad1.options)) {
            imu.resetYaw();
        }





        heading = move.drive.getExternalHeading();


        if (gamepad1.right_stick_x > 0.01 || gamepad1.right_trigger > 0.1) {
            targetAngle = 0;
            lockStatus = "unlocked";
        }
        else if(move.isPressed("x", gamepad1.x)) {
            if (lockStatus == "left"){ lockStatus = "unlocked";}
            else{ lockStatus = "left";}
        }
        else if(move.isPressed("y", gamepad1.y)) {
            if (lockStatus == "up"){ lockStatus = "unlocked";}
            else{ lockStatus = "up";}
        }
        else if(move.isPressed("a", gamepad1.a)){
            if (lockStatus == "down"){ lockStatus = "unlocked";}
            else{ lockStatus = "down";}
        }
        else if(move.isPressed("b", gamepad1.b)){
            if (lockStatus == "right"){ lockStatus = "unlocked";}
            else{ lockStatus = "right";}
        }

        if (lockStatus == "unlocked") {
            targetAngle = 0;
        }
        else if (lockStatus == "left") {
            targetAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + (-Math.PI / 2);
            if (lockStatus=="down"){
                targetAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + (Math.PI/2);
            }
        }
        else if (lockStatus == "up") {
            targetAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + (0);
        }
        else if (lockStatus == "down") {
            targetAngle = Math.atan2(Math.sin((imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))), Math.cos(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
            if (targetAngle > 0) {
                targetAngle -= Math.PI;
            }else {
                targetAngle += Math.PI;
            }
        }
        else if (lockStatus == "right")
        {targetAngle=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) +(Math.PI/2);}



        telemetry.addData("lockStatus: ", lockStatus);
        telemetry.addData("targetAngle: ", targetAngle);
        telemetry.addData("lift: ", manip.outtakeLiftMotor.getCurrentPosition());
        telemetry.addData("liftsense:", manip.liftTouchSensor.isPressed());


//        if(Math.abs(targetAngle)>=0 && Math.abs(targetAngle)<=Math.toRadians(2)){
//            targetAngle=0;
//        }

        if (gamepad1.right_bumper) {multiplier = 0.5;}
        else{multiplier=1;}

//        if (gamepad1.left_bumper) {invisStatus=true;}
//        else{invisStatus=false;}

        if (gamepad1.left_bumper) {
            augmentedSwerve = true;
        }
        else {
            augmentedSwerve = false;
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



        // driver 2 uses right bumper to toggle the outtake gate
        if (move.isPressed("rightBumper2", gamepad2.right_bumper)) {
            manip.gateToggle();
        }
        if (move.isPressed("leftBumper2", gamepad2.left_bumper)) {
            manip.gateToggle1();
        }

//        if(move.isPressed("rightBumper2", gamepad2.right_bumper)){
//            manip.climberLiftPower(.5);
//        } else if(move.isPressed("leftBumper2", gamepad2.left_bumper)){
//            manip.climberLiftPower(-.5);
//        } else{
//            manip.climberLiftPower(0);
//        }


        // uses dpad controls up and down to control the climber/hanger
//        if(gamepad2.right_stick_y > 0){
//            manip.leftClimberPower(.5);
//        } else if(gamepad2.right_stick_y < 0){
//            manip.leftClimberPower(-.5);
//        } else{
//            manip.leftClimberPower(0);
//        }

        if (gamepad2.dpad_up){
            manip.climberLeft.setPower(climberPower);
            manip.climberRight.setPower(climberPower);
            manip.winchLeft.setPower(-winchPower);
            manip.winchRight.setPower(winchPower);
        }
        else if (gamepad2.dpad_down){
            manip.climberLeft.setPower(-1 * climberPower);
            manip.climberRight.setPower(-1 * climberPower);
            manip.winchLeft.setPower(winchPower);
            manip.winchRight.setPower(-1 * winchPower);
        } else if (gamepad2.triangle) {
            manip.winchLeft.setPower(-winchPower);
            manip.winchRight.setPower(winchPower);
        }
        else if(gamepad2.x){
            manip.winchLeft.setPower(winchPower);
            manip.winchRight.setPower(-1 * winchPower);
        }
        else{
            manip.climberLeft.setPower(0);
            manip.climberRight.setPower(0);
            manip.winchLeft.setPower(0);
            manip.winchRight.setPower(0);
        }

        // When receiving power from gamepad2 that is greater than a certain threshold
        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            // it will move the lift to the certain power that the right joystick set
            if(manip.liftTouchSensor.isPressed() && gamepad2.right_stick_y > 0){
                manip.setOuttakeLiftPower(0);
            }
            else{
                manip.setOuttakeLiftPower(gamepad2.right_stick_y);
            }
        }
        else {
            //
            manip.setOuttakeLiftPower(0);
        }

        //uses right and left triggers to control lift
        if (gamepad2.right_trigger > 0.1){
            manip.setIntakePower(gamepad2.right_trigger);
        } else if (gamepad2.left_trigger > 0.1){
            //left trigger sets intake power in reverse
            manip.setIntakePower(-1.0 * gamepad2.left_trigger);
        } else {
            manip.setIntakePower(0);
        }

        if (gamepad2.share){
            manip.droneServo.setPower(0.8);
        }

//        if (Math.abs(gamepad2.left_stick_y) > 0.1){
//            manip.intakeLeftServo.setPower(gamepad2.left_stick_y);
//            manip.intakeRightServo.setPower(gamepad2.left_stick_y);
//        }
//        else{
//            manip.intakeLeftServo.setPower(0);
//            manip.intakeRightServo.setPower(0);
//        }

//        if (sensor.invisibleWallDetect()){
//            gamepad1.rumble(100);
//            gamepad2.rumble(100);
//
//        }
        if (gamepad2.circle) {
            manip.moveOuttakeLift(outtakeEncoderTicks);
        }

        telemetry.update();
    }}
