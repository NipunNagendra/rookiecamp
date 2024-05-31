package org.firstinspires.ftc.teamcode.rookiecamp.util;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive {
    DcMotor leftMotor;
    DcMotor rightMotor;
    private double x, y, h;
    private IMU imu;


    public Drive(HardwareMap hardwareMap, double x, double y, double h) {
        this.leftMotor = hardwareMap.dcMotor.get("leftMotor");
        this.rightMotor = hardwareMap.dcMotor.get("rightMotor");

        this.leftMotor.setDirection(DcMotor.Direction.FORWARD);
        this.rightMotor.setDirection(DcMotor.Direction.REVERSE);

        this.x = x;
        this.y = y;
        this.h = h;

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        imu.resetYaw();
    }

    public void setRelativePower(double leftPower, double rightPower) {
        this.leftMotor.setPower(Range.clip(leftPower, -1.0, 1.0));
        this.rightMotor.setPower(Range.clip(rightPower, -1.0, 1.0));
    }

    public void stop() {
        this.leftMotor.setPower(0);
        this.rightMotor.setPower(0);
    }
    public void update() {
        this.h = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        this.x += ((leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition())/ 2)*Math.cos(h);
        this.y +=  ((leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition())/2)*Math.sin(h);
        this.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getH()
    {
        update();
        return h;
    }

    public double getX()
    {
        update();
        return x;
    }

    public double getY()
    {
        update();
        return y;
    }

    public Pose getPose() {
        update();
        return new Pose(x, y, h);
    }

}

