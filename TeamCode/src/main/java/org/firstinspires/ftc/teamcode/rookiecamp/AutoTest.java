package org.firstinspires.ftc.teamcode.rookiecamp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rookiecamp.util.Drive;
import org.firstinspires.ftc.teamcode.rookiecamp.util.Pose2d;

public class AutoTest extends LinearOpMode {
    Drive robotDrive;
    Pose2d position = new Pose2d(0, 0, 0);
    double leftPower = 0, rightPower = 0;

    @Override
    public void runOpMode() {
        robotDrive = new Drive(hardwareMap);

        waitForStart();
//----------------------------------------------------------------------
        robotDrive.setRelativePower(1, 1);
        sleep(1000);
        robotDrive.setRelativePower(0, 0);
    }

}
