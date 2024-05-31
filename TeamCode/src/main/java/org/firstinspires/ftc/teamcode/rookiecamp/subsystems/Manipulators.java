package org.firstinspires.ftc.teamcode.rookiecamp.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Manipulators {
    DcMotorEx activeIntake;
    DcMotorEx flywheel;
    public static double currentThreshold = 0.1;

    public Manipulators(HardwareMap hardwareMap) {
        activeIntake = hardwareMap.get(DcMotorEx.class, "activeIntake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        activeIntake.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.FORWARD);

        activeIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setIntakePower(double power) {
        activeIntake.setPower(power);
    }
    public void setFlywheelPower(double power) {
        flywheel.setPower(power);
    }
    public boolean detectJam() {
        return Math.abs(activeIntake.getCurrent(CurrentUnit.AMPS)) < currentThreshold;
    }
    public void stop() {
        activeIntake.setPower(0);
        flywheel.setPower(0);
    }
}
