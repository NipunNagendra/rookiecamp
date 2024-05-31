package org.firstinspires.ftc.teamcode.rookiecamp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.rookiecamp.util.Drive;

public class teleop extends OpMode {
    Drive robotDrive;
    double drive;
    double turn;
    double leftPower = 0, rightPower = 0;

    @Override
    public void init() {
        robotDrive = new Drive(hardwareMap);
    }

    @Override
    public void loop() {
        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;

        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        robotDrive.setRelativePower(leftPower, rightPower);
    }

}
