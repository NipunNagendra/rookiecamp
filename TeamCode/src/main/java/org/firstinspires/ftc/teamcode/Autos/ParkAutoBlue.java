package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "ParkAutoBlue", group = "Autonomous")
public class ParkAutoBlue extends LinearOpMode{
    Manipulators manip;

    double startPoseX= 0;
    double startPoseY= 0;
    double startPoseAngle= 0;
    Pose2d startPose = new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseAngle));
    Pose2d posEstimate;

    //Add variables here

    public double strafeDistance=40;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);

        telemetry.addLine("Init Done");
//Add trajectories here
        TrajectorySequence park = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(strafeDistance)
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

