package org.firstinspires.ftc.teamcode.libs;

import android.graphics.Color;
import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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
    public Servo droneServo;
    public DcMotor leftClimberMotor;
    public DcMotor rightClimberMotor;

    public DcMotor intakeMotor;

    public DcMotor outtakeLiftMotor;
    double outtakeLiftTicks = 537.7;
    double newOuttakeLiftTarget;

    public HashMap<String, Boolean> buttons = new HashMap<String, Boolean>();
    public String officialColor;

    public static double outtakeServoPos1 = 0.15;
    public static double outtakeServoPos2 = 0;

    //sensor shi
    public RevColorSensorV3 upperCS;
    public RevColorSensorV3 lowerCS;
    public DistanceSensor ds;

    public static double restDistance = 106;
    public static double sensorMarginalThreshold = 0.2;
    public TouchSensor liftTouchSensor;

    public Manipulators(HardwareMap hardwareMap) {
        this.robot = hardwareMap;

        // declaring climber/hanging motors
        leftClimberMotor = hardwareMap.get(DcMotor.class, "leftClimber");
        rightClimberMotor = hardwareMap.get(DcMotor.class, "rightClimber");
        //outtakeServo = hardwareMap.get(Servo.class, "outtakeServo");

        // setting climber/hanging motor direction
        leftClimberMotor.setDirection(DcMotor.Direction.REVERSE);
        rightClimberMotor.setDirection(DcMotor.Direction.FORWARD);

        // setting break mode for climber/hanging/outtake motors
        leftClimberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightClimberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //outtakeServo = hardwareMap.get(Servo.class, "outtakeServo");

        outtakeLiftMotor = hardwareMap.get(DcMotor.class, "outtakeLift");
        outtakeLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        outtakeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //declaring intake motor
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        //liftTouchSensor = hardwareMap.get(TouchSensor.class, "liftTouchSensor");

        //droneServo = hardwareMap.get(Servo.class, "droneServo");

        //Sensor Declaration
       // lowerCS = hardwareMap.get(RevColorSensorV3.class, "lowerCS");
       // ds = hardwareMap.get(DistanceSensor.class, "ds");
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

    //Gate output toggle method
//    public void gateToggle(boolean outtakeServoStatus) {
//        //Checking the status of the outtake servo
//        if (outtakeServoStatus == false){
//            outtakeServo.setPosition(outtakeServoPos1); //0.15
//        } else if (outtakeServoStatus){
//            outtakeServo.setPosition(outtakeServoPos2); //0
//        }
//    }

    // sets power to control climber/hanging motors
    public void climberLiftPower(double motorPower){
        leftClimberMotor.setPower(motorPower);
        rightClimberMotor.setPower(motorPower);}

    // changes the position of the servo to launch the drone
    public void droneLaunch() {
        droneServo.setPosition(.60);
    }


    public void setIntakePower(double power){intakeMotor.setPower(power);}

    public void moveOuttakeLift(int encoderTicks){
        outtakeLiftMotor.setTargetPosition(encoderTicks);
        outtakeLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeLiftMotor.setPower(0);
    }

//    public void bottomOutLift(){
//        while(!liftTouchSensor.isPressed()){
//            outtakeLiftMotor.setPower(-.9);
//        }
//        outtakeLiftMotor.setPower(0);
//    }
}
