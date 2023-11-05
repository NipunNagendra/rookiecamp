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
        SPIKEMARK,
        SCORE_PURPLE,
        BACKDROP,
        STACK,
        PARK
    }

    double startPoseX = 0;
    double startPoseY = 0;
    double startPoseAngle = 0;
    Pose2d startPose = new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseAngle));
    Pose2d posEstimate;

    //Add variables here
    double spikemarkRightX = 0;
    double spikemarkRightY = 0;
    double spikemarkRightPoseAngle = 0;

    double spikemarkLeftX = 0;
    double spikemarkLeftY = 0;
    double spikemarkLeftPoseAngle = 0;

    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addLine("Init Done");
//Add trajectories here
        TrajectorySequence spikemark = drive.trajectorySequenceBuilder(startPose)
                //Right
                .lineToLinearHeading(new Pose2d(spikemarkRightX,spikemarkRightY,spikemarkRightPoseAngle))

                .build();




        telemetry.addLine("trajectories built!!!");

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            posEstimate = drive.getPoseEstimate();

            switch(currentState){
                case IDLE:
                    currentState = State.SPIKEMARK;

                case SPIKEMARK:
                    drive.followTrajectorySequence(spikemark);
                    currentState = State.IDLE;
                    break;
            }
        }
    }

}

