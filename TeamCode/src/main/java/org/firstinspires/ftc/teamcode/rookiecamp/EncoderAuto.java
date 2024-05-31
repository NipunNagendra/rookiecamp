package org.firstinspires.ftc.teamcode.rookiecamp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rookiecamp.subsystems.Drive;
import org.firstinspires.ftc.teamcode.rookiecamp.subsystems.Manipulators;

@Autonomous
public class EncoderAuto extends LinearOpMode {

    Drive robotDrive;
    Manipulators robotManipulators;
    double drive;
    double turn;
    double leftPos = 0, rightPos = 0;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robotDrive = new Drive(hardwareMap, 0, 0, 0);
        robotManipulators = new Manipulators(hardwareMap);

        robotDrive.leftMotor.setTargetPosition(0);
        robotDrive.rightMotor.setTargetPosition(0);

        robotDrive.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotDrive.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotDrive.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotDrive.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotDrive.setRelativePower(1,1);
        waitForStart();

        leftPos += 3000;
        rightPos += 3000;

        robotDrive.leftMotor.setTargetPosition((int) leftPos);
        robotDrive.leftMotor.setTargetPosition((int) rightPos);


        sleepMilli(3000);

        leftPos += 600;
        rightPos += -600;

        robotDrive.leftMotor.setTargetPosition((int) leftPos);
        robotDrive.leftMotor.setTargetPosition((int) rightPos);

        sleepMilli(3000);


    }

    private void sleepMilli(double time) {
        timer.reset();
        while (timer.milliseconds()<time) {

        }
    }
}
