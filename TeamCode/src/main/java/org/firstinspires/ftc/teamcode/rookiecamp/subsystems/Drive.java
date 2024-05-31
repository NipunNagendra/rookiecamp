package org.firstinspires.ftc.teamcode.rookiecamp.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.rookiecamp.util.Pose;

public class Drive {
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    private double x, y, h;
    public IMU imu;
    private double lPos, rPos;


    public Drive(HardwareMap hardwareMap, double x, double y, double h) {
        this.leftMotor = hardwareMap.dcMotor.get("leftMotor");
        this.rightMotor = hardwareMap.dcMotor.get("rightMotor");
        this.imu = hardwareMap.get(BNO055IMUNew.class, "imu");

        this.leftMotor.setDirection(DcMotor.Direction.FORWARD);
        this.rightMotor.setDirection(DcMotor.Direction.REVERSE);

        this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lPos = 0;
        rPos = 0;

        this.x = x;
        this.y = y;
        this.h = h;

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
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
        this.x += (((lPos-leftMotor.getCurrentPosition()) + (rPos-rightMotor.getCurrentPosition()))/ 2.0)*Math.cos(h);
        this.y +=  (((lPos-leftMotor.getCurrentPosition()) + (rPos-rightMotor.getCurrentPosition()))/2.0)*Math.sin(h);
        lPos = leftMotor.getCurrentPosition();
        rPos = rightMotor.getCurrentPosition();
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
        return new Pose((x/384.5) * (Math.PI), (y/384.5* (Math.PI)), h);
    }

}

