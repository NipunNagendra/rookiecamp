package org.firstinspires.ftc.teamcode.libs;

import static java.lang.Thread.sleep;

import android.graphics.Color;
import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;

@Config
public class Manipulators {
    HardwareMap robot;
    public Servo outtakeServo;
    public CRServo intakeRightServo;
    public CRServo intakeLeftServo;
    public CRServo droneServo;
    public DcMotor leftClimberMotor;
    public DcMotor rightClimberMotor;

    public DcMotor intakeMotor;

    public DcMotor outtakeLiftMotor;
    double outtakeLiftTicks = 537.7;
    double newOuttakeLiftTarget;

    public HashMap<String, Boolean> buttons = new HashMap<String, Boolean>();
    public String officialColor;

    public static double outtakeServoPos1 = 0.23;
    public static double outtakeServoPos2 = 0.4;

    public static double droneLaunchPos = 0.5;

    //intake servo shi
    public static double intakeServoPos1 = 0;
    public static double intakeServoPos2 = 0;
    public static double intakeServoPos3 = 0;
    public static double intakeServoPos4 = 0;
    public static double intakeServoPos5 = 0;

    //sensor shi
    public RevColorSensorV3 upperCS;
    public RevColorSensorV3 lowerCS;
    public DistanceSensor ds;

    public static double restDistance = 106;
    public static double sensorMarginalThreshold = 0.2;
    public RevTouchSensor liftTouchSensor;

    public Manipulators(HardwareMap hardwareMap) {
        this.robot = hardwareMap;

        // declaring climber/hanging motors
        leftClimberMotor = hardwareMap.get(DcMotor.class, "leftClimberMotor");
        rightClimberMotor = hardwareMap.get(DcMotor.class, "rightClimberMotor");
        outtakeServo = hardwareMap.get(Servo.class, "outtakeServo");

        //setting climber/hanging motor direction
        leftClimberMotor.setDirection(DcMotor.Direction.FORWARD);
        rightClimberMotor.setDirection(DcMotor.Direction.REVERSE);

        // setting break mode for climber/hanging/outtake motors
        leftClimberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightClimberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        outtakeServo = hardwareMap.get(Servo.class, "outtakeServo");

        outtakeLiftMotor = hardwareMap.get(DcMotor.class, "outtakeLiftMotor");
        outtakeLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        outtakeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // setting servos for right and left to hardware map
        intakeRightServo = hardwareMap.get(CRServo.class, "intakeRightServo");
        intakeLeftServo = hardwareMap.get(CRServo.class, "intakeLeftServo");

        intakeRightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        //declaring intake motor
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        liftTouchSensor = hardwareMap.get(RevTouchSensor.class, "RevTouchSensor");
//
//        droneServo = hardwareMap.get(CRServo.class, "droneServo");

        //Sensor Declaration
//        lowerCS = hardwareMap.get(RevColorSensorV3.class, "lowerCS");
//       ds = hardwareMap.get(DistanceSensor.class, "ds");
    }


    public boolean isColor(String color){
        boolean output = false;

        String colorWas = officialColor;

        if (color != colorWas){
            output = true;
            officialColor=color;
        }

        return output;
    }
    //returned normalized colors for the upper color sensor, and the color it detects
    public Pair<float[], String> getNormalizedColor(){
        int red = upperCS.red();
        int green = upperCS.green();
        int blue = upperCS.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);
        float hue = hsv[0];
        String color;

        if (hue >= 190 && hue <= 300) {
            color = "purple";
        } else if (hue >= 105 && hue <= 150) {
            color = "green";
        } else if (hue >= 16 && hue <= 100) {
            color = "yellow";
        } else {
            color = "weird color";
        }

        return new Pair<>(hsv,color);

    }

    public boolean beamBreakUpper(){
        if (Math.abs(upperCS.getDistance(DistanceUnit.MM)-restDistance) /
                restDistance>=sensorMarginalThreshold){
            return true;
        }
        else {
            return false;
        }

    }
    public boolean beamBreakLower(){
        if (Math.abs(lowerCS.getDistance(DistanceUnit.MM)-restDistance) /
                restDistance>=sensorMarginalThreshold){
            return true;
        }
        else {
            return false;
        }

    }

    // Creating method for Lift outtake
    public void setOuttakeLiftPower(double liftPower) {
        // setting power to lift
        outtakeLiftMotor.setPower(liftPower);
    }

//    Gate output toggle method
    public void gateToggle() {
        //Checking the status of the outtake servo
        //outtakeServo.setDirection(Servo.Direction.FORWARD);
        Thread outtakeMove = new Thread() {
            @Override
              public void run(){
                outtakeServo.setPosition(outtakeServoPos2);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                outtakeServo.setPosition(outtakeServoPos1);
            }

        };
        outtakeMove.start();
    }
    public void gateToggle1(){
       // outtakeServo.setDirection(Servo.Direction.REVERSE);
        Thread outtakeMoveBoth = new Thread() {
            @Override
            public void run(){
                outtakeServo.setPosition(outtakeServoPos2);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                outtakeServo.setPosition(outtakeServoPos1);
                try {
                    Thread.sleep(700);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                outtakeServo.setPosition(outtakeServoPos2);
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                outtakeServo.setPosition(outtakeServoPos1);

            }

        };
        outtakeMoveBoth.start();    }


    // Intake servo toggle method
//    public void intakeToggle(boolean intakeServoStatus) {
//        // intakeServoStatus = true -> down
//        // false -> up
//        if (intakeServoStatus == false) {
//            intakeRightServo.setPosition(intakeServoPos5);
//            intakeLeftServo.setPosition(intakeServoPos5);
//        } else if (intakeServoStatus) {
//            intakeRightServo.setPosition(intakeServoPos1);
//            intakeLeftServo.setPosition(intakeServoPos1);
//        }
//    }

    // sets the intake servo to different positions based on the number of pixels/height of the stack
//    public void intakeServoHeights(int numOfPixels) {
//        if (numOfPixels == 1) {
//            intakeRightServo.setPosition(intakeServoPos1);
//            intakeLeftServo.setPosition(intakeServoPos1);
//        } else if (numOfPixels == 2) {
//            intakeRightServo.setPosition(intakeServoPos2);
//            intakeLeftServo.setPosition(intakeServoPos2);
//        } else if (numOfPixels == 3) {
//            intakeRightServo.setPosition(intakeServoPos3);
//            intakeLeftServo.setPosition(intakeServoPos3);
//        } else if (numOfPixels == 4) {
//            intakeRightServo.setPosition(intakeServoPos4);
//            intakeLeftServo.setPosition(intakeServoPos4);
//        } else if (numOfPixels == 5) {
//            intakeRightServo.setPosition(intakeServoPos5);
//            intakeLeftServo.setPosition(intakeServoPos5);
//        } else {
//            // invalid
//        }
//    }
    // sets power to control climber/hanging motors
    public void climberLiftPower(double motorPower){
        leftClimberMotor.setPower(motorPower);
        rightClimberMotor.setPower(motorPower);}

    public void leftClimberPower(double motorPower){
        leftClimberMotor.setPower(motorPower);
    }
    public void rightClimberPower(double motorPower){
        rightClimberMotor.setPower(motorPower);
    }

    // changes the position of the servo to launch the drone
//    public void droneLaunch() {
//        droneServo.setPosition(droneLaunchPos);
//    }


    public void setIntakePower(double power){intakeMotor.setPower(power);}

    public void moveOuttakeLift(int encoderTicks){
        outtakeLiftMotor.setTargetPosition(encoderTicks);
        outtakeLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeLiftMotor.setPower(0.5);

    }

    public void bottomOutLift(){
        while(!liftTouchSensor.isPressed()){
            outtakeLiftMotor.setPower(-.9);
        }
        outtakeLiftMotor.setPower(0);
    }
}
