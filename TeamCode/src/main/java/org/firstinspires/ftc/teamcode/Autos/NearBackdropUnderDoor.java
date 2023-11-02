package org.firstinspires.ftc.teamcode.Autos;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "NearBackdropUnderDoor", group = "Autonomous")


public class NearBackdropUnderDoor extends LinearOpMode{


    enum State{
        IDLE,
        INITIAL_MOVE,
        MIDDLE_SPIKEMARK,
        RIGHT_SPIKEMARK,
        LEFT_SPIKEMARK,
        BACKDROP,
        STACK,
        PARK
    }

    double startPoseX= 0;
    double startPoseY= 0;
    double startPoseAngle= 0;
    Pose2d startPose = new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseAngle));
    Pose2d posEstimate;

    //Add variables here
    double initialPoseX = 0;
    double initialPoseY = 0;
    double initialPoseAngle = 0;

    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addLine("Init Done");
//Add trajectories here
        TrajectorySequence initialMove = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(initialPoseX,initialPoseY,initialPoseAngle))
                .build();

        telemetry.addLine("trajectories built!!!");

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            posEstimate = drive.getPoseEstimate();

            switch(currentState){
                case IDLE:
                    currentState = State.INITIAL_MOVE;

                case INITIAL_MOVE:
                    drive.followTrajectorySequence(initialMove);
                    currentState = State.IDLE;
                    break;
            }
        }
    }

}

