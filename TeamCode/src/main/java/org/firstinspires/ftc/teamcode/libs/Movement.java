package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;


public class Movement {

    HardwareMap robot;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor outtakeMotor;
    private DcMotor climberleft;
    private DcMotor climberRight;
    private DcMotor inttakeMotor;

    public HashMap<String, Boolean> buttons = new HashMap<String, Boolean>();

    public Movement(HardwareMap hardwareMap) {
        this.robot = hardwareMap;

        FL = robot.get(DcMotor.class, "FL");
        FR = robot.get(DcMotor.class, "FR");
        BL = robot.get(DcMotor.class, "BL");
        BR = robot.get(DcMotor.class, "BR");

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //motors are reversed than what they shld be
        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);
    }

    public double[] holonomicDrive(double leftX, double leftY, double rightX) {
        double[] motorpowers = new double[4];
        motorpowers[0] = leftY - leftX - rightX;
        motorpowers[1] = leftY + leftX + rightX;
        motorpowers[2] = leftY + leftX - rightX;
        motorpowers[3] = leftY - leftX + rightX;
        return motorpowers;
    }

    public double[] fieldDrive(double leftX, double leftY, double rightX, double imu) {
        double botHeading = imu;
        double rotX = leftX * Math.cos(-botHeading) - leftY * Math.sin(-botHeading);
        double rotY = leftX * Math.sin(-botHeading) + leftY * Math.cos(-botHeading);

        rotX = rotX * 1.1;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightX), 1);

        return new double[]{(rotY - rotX - rightX) / denominator, (rotY + rotX + rightX) / denominator, (rotY + rotX - rightX) / denominator, (rotY - rotX + rightX) / denominator};
    }

    public void setPowers(double[] motorPower, double multiplier) {

        FR.setPower(motorPower[0]*multiplier);
        FL.setPower(motorPower[1]*multiplier);
        BR.setPower(motorPower[2]*multiplier);
        BL.setPower(motorPower[3]*multiplier);
    }

    public void setPowers(double fr, double fl, double br, double bl) {

        FR.setPower(fr);
        FL.setPower(fl);
        BR.setPower(br);
        BL.setPower(bl);
    }

    public boolean isPressed(String name, boolean button){
        boolean output = false;

        //If the hashmap doesn't already contain the key
        if (!buttons.containsKey(name)) {
            buttons.put(name, false);
        }

        boolean buttonWas = buttons.get(name);
        if (button != buttonWas && button == true){
            output = true;
        }

        buttons.put(name, button);

        return output;
}
    public boolean isColor(String color, boolean state){
        boolean output = false;

        //If the hashmap doesn't already contain the key
        if (!buttons.containsKey(color)) {
            buttons.put(color, false);
        }

        boolean buttonWas = buttons.get(color);
        if (state != buttonWas && state == true){
            output = true;
        }

        buttons.put(color, state);

        return output;
    }
}

