package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.testing.BluePipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "BackdropAutoBlue", group = "Autonomous")
public class BackdropAutoBlue extends LinearOpMode {

    enum State{
        IDLE,
        SCORE_PURPLE,
        SCORE_YELLOW,
        PARK,
        STOP
    }

    /* TODO: ENTER COORDINATES FOR ALL POSITIONS
    *   CHANGE ALL VARIABLES TO PUBLIC STATIC
    *   REFACTOR ALL VARIABLES TO MORE DESCRIPTIVE NAMES
    */


    //coordinates for starting position (0, 0, 0)
    public static double startPoseX= 11.35845302224215;
    public static double startPoseY= -65.13672263931143;
    public static double startPoseAngle= 90;

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseAngle));

    Pose2d posEstimate;

    //coordinates for left spike position
    public static double spike1X = 11.35845302224215;
    public static double spike1Y = 34.44644728121096;
    public static double spike1Angle = Math.toRadians(0);

    //coordinates for middle spike position
    public static double spike2X = 10.043017898968719;
    public static double spike2Y = -28.911245972675605;
    public static double spike2Angle = Math.toRadians(90);

    //coordinates for right spike position
    public static double spike3X = 11.35845302224215;
    public static double spike3Y = 34.44644728121096;
    public static double spike3Angle = Math.toRadians(180);

    //coordinates for BackDrop position
    public static double backdropMiddleX = 45;
    public static double backdropMiddleY = 34;
    public static double backdropMiddleAngle = Math.toRadians(180);

    public static double strafeToTag = 8;

    public static double temporalMarkerTimeUP = 1.5;
    public static double temporalMarkerTimeDOWN = 0.75;


    public static int outtakeEncoderTicks = 2500;
    public static int outtakeOG = 0;

    // stores the result of Vision locally

    public static BluePipeline.Location positionOfVisionPixel;
    public static double casePos;
    State currentState = State.IDLE;
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        BluePipeline vision =  new BluePipeline(telemetry);

        telemetry.addLine("Init Done");
        drive.setPoseEstimate(startPose);
        //still need to enter values for these
        TrajectorySequence scorePurpleLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(spike1X, spike1Y, spike1Angle))
                .build();

        //still need to enter values for these
        TrajectorySequence scorePurpleMiddle = drive.trajectorySequenceBuilder(startPose)
                .back(7)
                .turn(Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(spike2X, spike2Y, spike2Angle))
                .build();

        //still need to enter values for these
        TrajectorySequence scorePurpleRight = drive.trajectorySequenceBuilder(startPose)
                .back(10)
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(spike3X, spike3Y, spike3Angle))
                .forward(3)
                .build();

        TrajectorySequence backDropLeft = drive.trajectorySequenceBuilder(scorePurpleLeft.end())
                .strafeLeft(27)
                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                .addTemporalMarker(temporalMarkerTimeUP, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicks);
                })
                .strafeRight(strafeToTag)
                .build();

        TrajectorySequence backDropMiddle = drive.trajectorySequenceBuilder(scorePurpleMiddle.end())
                .back(7)
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                .forward(5)
                .strafeRight(strafeToTag)
                .build();

        TrajectorySequence backDropRight = drive.trajectorySequenceBuilder(scorePurpleRight.end())
                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                .addTemporalMarker(temporalMarkerTimeUP, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicks);
                })
                .strafeLeft(strafeToTag)
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(backDropLeft.end())
                .addTemporalMarker(temporalMarkerTimeDOWN, () -> {
                    manip.moveOuttakeLift(outtakeOG);
                })
                .strafeRight(18)
                .back(15)
                .build();

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(backDropMiddle.end())
                .addTemporalMarker(temporalMarkerTimeDOWN, () -> {
                    manip.moveOuttakeLift(outtakeOG);
                })
                .strafeRight(16)
                .turn(Math.toRadians(90))
                .strafeLeft(15)
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(backDropRight.end())
                .addTemporalMarker(temporalMarkerTimeDOWN, () -> {
                    manip.moveOuttakeLift(outtakeOG);
                })
                .strafeRight(32)
                .back(15)
                .build();

        telemetry.addLine("trajectories built!!!");
        telemetry.addData("cpos", vision.getLocation());
        telemetry.update();
        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            posEstimate = drive.getPoseEstimate();

            switch(currentState){
                case IDLE:
                    currentState = State.SCORE_PURPLE;

                case SCORE_PURPLE:
                    positionOfVisionPixel = vision.getLocation();
                    if (BluePipeline.positionMain == "left") {
                        telemetry.addLine("going left");
                        casePos=1;
                        drive.followTrajectorySequence(scorePurpleLeft);
                    } else if (BluePipeline.positionMain == "middle") {
                        telemetry.addLine("going middle");
                        casePos=2;
                        drive.followTrajectorySequence(scorePurpleMiddle);
                    } else {
                        telemetry.addLine("goingright");
                        casePos=3;
                        drive.followTrajectorySequence(scorePurpleRight);
                    }
                    posEstimate = drive.getPoseEstimate();
                    manip.setIntakePower(-1);
                    sleep(1600);

                    manip.setIntakePower(0);
                    currentState = State.PARK;
                    break;

                case SCORE_YELLOW:
                    if (casePos==1) {
                        drive.followTrajectorySequence(backDropLeft);
                    } else if (casePos==2) {
                        drive.followTrajectorySequence(backDropMiddle);
                    } else {
                        drive.followTrajectorySequence(backDropMiddle);
                    }
                    manip.gateToggle();
                    currentState = BackdropAutoBlue.State.STOP;
                    break;

                case PARK:
                    if (casePos==1) {
                        drive.followTrajectorySequence(parkLeft);
                    } else if (casePos==2) {
                        drive.followTrajectorySequence(parkMiddle);
                    } else {
                        drive.followTrajectorySequence(parkRight);
                    }
                    currentState = BackdropAutoBlue.State.STOP;
                    break;

                case STOP:
                    break;
            }
        }

    }

}
