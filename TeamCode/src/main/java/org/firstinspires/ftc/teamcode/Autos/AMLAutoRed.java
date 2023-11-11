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
@Autonomous(name = "NipunAuto", group = "Autonomous")
public class AMLAutoRed extends LinearOpMode{
    Manipulators manip;
    enum State{
        IDLE,
        SCORE_PURPLE,
        PARK
    }
    public String visionPosition;
    double startPoseX= 0;
    double startPoseY= 0;
    double startPoseAngle= 0;
    Pose2d startPose = new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseAngle));
    Pose2d posEstimate;

    //Add variables here

    public double middle_score_forward=0;
    public double middle_strafe_value=0;


    State currentState = State.SCORE_PURPLE;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);

        telemetry.addLine("Init Done");
//Add trajectories here
        Trajectory middle_score = drive.trajectoryBuilder(startPose)
                .forward(middle_score_forward)
                .build();
        TrajectorySequence middle_park =drive.trajectorySequenceBuilder(middle_score.end())
                .back(middle_score_forward)
                .strafeRight(middle_strafe_value)
                .build();

        TrajectorySequence score_left = drive.trajectorySequenceBuilder(new Pose2d(11.06, -63.42, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(10.38, -33.86, Math.toRadians(180.00)))
                .build();

        TrajectorySequence parkleft = drive.trajectorySequenceBuilder(score_left.end())
                .lineToConstantHeading(new Vector2d(37.69, -62.07))
                .lineToConstantHeading(new Vector2d(62.52, -62.75))
                        .build();



        telemetry.addLine("trajectories built!!!");

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            posEstimate = drive.getPoseEstimate();

            switch(currentState){
                case IDLE:
                    break;

                case PARK:
                    if(visionPosition=="left") {
                        drive.followTrajectorySequence(score_left);

                        manip.setIntakePower(-1);
                        sleep(2000);
                        manip.setIntakePower(0);

                        drive.followTrajectorySequence(parkleft);
                    }
                    else if (visionPosition=="middle"){
                        drive.followTrajectory(middle_score);

                        manip.setIntakePower(-1);
                        sleep(2000);
                        manip.setIntakePower(0);

                        drive.followTrajectorySequence(middle_park);

                    }
                    currentState = State.IDLE;
                    break;
            }
        }
    }

}
