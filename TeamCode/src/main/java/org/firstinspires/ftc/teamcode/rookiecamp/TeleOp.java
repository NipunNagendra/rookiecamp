package org.firstinspires.ftc.teamcode.rookiecamp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.rookiecamp.subsystems.Drive;
import org.firstinspires.ftc.teamcode.rookiecamp.subsystems.Manipulators;
import org.firstinspires.ftc.teamcode.rookiecamp.util.Pose;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Rookie Camp teleop", group = "rookiecamp")
public class TeleOp extends OpMode {
    Drive robotDrive;
    Manipulators robotManipulators;
    double drive;
    double turn;
    double leftPower = 0, rightPower = 0;

    @Override
    public void init() {
        robotDrive = new Drive(hardwareMap, 0, 0, 0);
        robotManipulators = new Manipulators(hardwareMap);
    }

    @Override
    public void loop() {
        drive = -gamepad1.left_stick_y;
        turn = -gamepad1.right_stick_x*0.5;

        leftPower = drive + turn;
        rightPower = drive - turn;

        leftPower = Range.clip(leftPower, -1.0, 1.0);
        rightPower = Range.clip(rightPower, -1.0, 1.0);

        robotDrive.setRelativePower(leftPower, rightPower);

        if (gamepad1.left_bumper){
            // outtake
            robotManipulators.setIntakePower(-1);
        } else if (!robotManipulators.detectJam()){
            // intake
            robotManipulators.setIntakePower(gamepad1.left_trigger);
        }
        else{
            robotManipulators.setIntakePower(0);
        }

        if (gamepad1.right_bumper){
            // push back in
            robotManipulators.setFlywheelPower(-1);
        } else {
            // shoot
            robotManipulators.setFlywheelPower(gamepad1.right_trigger);
        }

        Pose poseEstimate = robotDrive.getPose();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("h", poseEstimate.getHeading());
        telemetry.update();
        robotDrive.update();
    }
}
