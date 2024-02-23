package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="GuardMode", group="TeleOp")
public class GuardMode extends LinearOpMode {
   public static double xyParameter = .2;
    public static double headingParameter = .2;
    double lockThreshold = 0;
    Pose2d lockLocation = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

            lockTo(lockLocation, drive);
            drive.update();

        }
    }

    public void lockTo(Pose2d targetPos, SampleMecanumDrive drive) {
        Pose2d currPos = drive.getPoseEstimate();
        Pose2d difference = targetPos.minus(currPos);
        Vector2d xy = difference.vec().rotated(-currPos.getHeading());
        double heading = Angle.normDelta(targetPos.getHeading()) - Angle.normDelta(currPos.getHeading());

        drive.setWeightedDrivePower(new Pose2d(xy.times(xyParameter), heading* headingParameter));
    }}

