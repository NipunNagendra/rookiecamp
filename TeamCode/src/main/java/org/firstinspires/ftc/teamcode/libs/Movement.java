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

    public final double FL_PROPORTION;
    public final double FR_PROPORTION;
    public final double BL_PROPORTION;
    public final double BR_PROPORTION;

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

        FL_PROPORTION = 1;
        FR_PROPORTION = 1;
        BL_PROPORTION = 1;
        BR_PROPORTION = 1;
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

        FR.setPower(motorPower[0]*multiplier* FR_PROPORTION);
        FL.setPower(motorPower[1]*multiplier* FL_PROPORTION);
        BR.setPower(motorPower[2]*multiplier* BR_PROPORTION);
        BL.setPower(motorPower[3]*multiplier* BL_PROPORTION);
    }

    public void setPowers(double fr, double fl, double br, double bl) {

        FR.setPower(fr);
        FL.setPower(fl);
        BR.setPower(br);
        BL.setPower(bl);
    }

    public void strafeRight(double power){
        FR.setPower(-power);
        FL.setPower(power);
        BR.setPower(power);
        BL.setPower(-power);
    }
    public void strafeLeft(double power){
        FR.setPower(power);
        FL.setPower(-power);
        BR.setPower(-power);
        BL.setPower(power);
    }

    public void kill(){
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
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
}

