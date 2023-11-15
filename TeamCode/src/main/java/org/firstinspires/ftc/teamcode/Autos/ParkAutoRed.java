package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "ParkAutoRed", group = "Autonomous")
public class ParkAutoRed extends LinearOpMode{
    Manipulators manip;

    double startPoseX= 0;
    double startPoseY= 0;
    double startPoseAngle= 0;
    Pose2d startPose = new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseAngle));
    Pose2d posEstimate;

    //Add variables here

    public double distance =40;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);

        telemetry.addLine("Init Done");
//Add trajectories here
        TrajectorySequence park = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(distance)
                .build();

        telemetry.addLine("trajectories built!!!");

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            posEstimate = drive.getPoseEstimate();
            drive.followTrajectorySequence(park);
            manip.intakeMotor.setPower(-1);
            sleep(7000);
            manip.intakeMotor.setPower(0);
            break;
        }
    }
}


