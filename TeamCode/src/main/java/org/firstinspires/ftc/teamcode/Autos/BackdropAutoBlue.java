package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.testing.BluePipeline;
import org.firstinspires.ftc.teamcode.testing.RedPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

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


    //This is code for BackdropAutoBlue
    //coordinates for starting position (0, 0, 0)
    public static double startPoseX = 11.35845302224215;
    public static double startPoseY = 65.13672263931143;
    public static double startPoseAngle = Math.toRadians(90);

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    public static double moveBackwards3 = 31;
    public static double moveBackwardsTowardBackDrop = 34;
    public static double turn3 = 90;

    //coordinates for right spike position
    public static double spike3X = 11.35845302224215;
    public static double spike3Y = 30.44644728121096;
    public static double spike3Angle = Math.toRadians(180);

    //coordinates for left spike position
    public static double spike1X = 11.35845302224215;
    public static double spike1Y = 34.44644728121096;
    public static double spike1Angle = Math.toRadians(0);

    //coordinates for middle spike position
    public static double spike2X =  11.35845302224215;
    public static double spike2Y = 34.44644728121096;
    public static double spike2Angle = Math.toRadians(270);

    public static double backdropMiddleX = 45;
    public static double backdropMiddleY = 34;
    public static double backdropMiddleAngle = Math.toRadians(180);

    public static double backdropLeftX = 44;
    public static double backdropLeftY = 44;
    public static double backdropLeftAngle = Math.toRadians(180);

    public static double backdropRightX = 45;
    public static double backdropRightY = 29;
    public static double backdropRightAngle = Math.toRadians(180);
    public static double preTrussX = -38.15845302224215;
    public static double trussX = 15;
    public static double trussY = -55.93672263931143;
    public static double trussAngle = Math.toRadians(180);

    public static double strafeForPark = 20;
    public static double backdropRightStrafe = 8;

    public static double casenum=1;


    public static String myPosition;

    public static double strafeToTag = 8;

    public static double temporalMarkerTimeDOWN = 1.5;
    public static double temporalMarkerTimeUP = 5;


    public static int outtakeEncoderTicks = 2000;
    public static int outtakeOG = 0;

    // stores the result of Vision locally

    public static BluePipeline.Location positionOfVisionPixel;
    public static double casePos;
    State currentState = State.IDLE;
    OpenCvWebcam camera;
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
                .forward(3)
                .build();

        //still need to enter values for these
        TrajectorySequence scorePurpleMiddle = drive.trajectorySequenceBuilder(startPose)
                .back(7)
                .lineToLinearHeading(new Pose2d(spike2X, spike2Y, spike2Angle))
                .forward(3)
                .build();

        //still need to enter values for these
        TrajectorySequence scorePurpleRight = drive.trajectorySequenceBuilder(startPose)
                .back(10)
                // .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(spike3X, spike3Y, spike3Angle))
                .forward(3)
                .build();

        TrajectorySequence backDropLeft = drive.trajectorySequenceBuilder(scorePurpleLeft.end())
                .back(5)
                .strafeLeft(20)
                .lineToLinearHeading(new Pose2d(backdropLeftX, backdropLeftY, backdropLeftAngle))
                .addTemporalMarker(temporalMarkerTimeUP, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicks);
                })
                .build();

        TrajectorySequence backDropMiddle = drive.trajectorySequenceBuilder(scorePurpleMiddle.end())
                .back(7)
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                .addTemporalMarker(temporalMarkerTimeUP, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicks);
                })
                .build();

        TrajectorySequence backDropRight = drive.trajectorySequenceBuilder(scorePurpleRight.end())
                .lineToLinearHeading(new Pose2d(backdropRightX, backdropRightY, backdropRightAngle))
                .addTemporalMarker(temporalMarkerTimeUP, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicks);
                })
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(backDropLeft.end())
                .forward(5)
                .addTemporalMarker(temporalMarkerTimeDOWN, () -> {
                    manip.moveOuttakeLift(outtakeOG);
                })
                .strafeRight(15)
                .turn(Math.toRadians(90))
                .strafeLeft(15)
                .build();

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(backDropMiddle.end())
                .forward(5)
                .addTemporalMarker(temporalMarkerTimeDOWN, () -> {
                    manip.moveOuttakeLift(outtakeOG);
                })
                .strafeRight(24)
                .turn(Math.toRadians(90))
                .strafeLeft(15)
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(backDropRight.end())
                .forward(5)
                .addTemporalMarker(temporalMarkerTimeDOWN, () -> {
                    manip.moveOuttakeLift(outtakeOG);
                })
                .strafeRight(30)
                .turn(Math.toRadians(90))
                .strafeLeft(15)
                .build();

        telemetry.addLine("trajectories built!!!");
        telemetry.addData("cpos", vision.getLocation());
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "identifyier","teamcode");
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        BluePipeline detectBlue = new BluePipeline(telemetry);
        camera.setPipeline(detectBlue);

        camera.setMillisecondsPermissionTimeout(5000);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {



            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        waitForStart();


        camera.stopStreaming();

        while(!isStopRequested() && opModeIsActive()){
            posEstimate = drive.getPoseEstimate();

            switch(currentState){
                case IDLE:
                    currentState = State.SCORE_PURPLE;

                case SCORE_PURPLE:
                    positionOfVisionPixel = vision.getLocation();
                    if (BluePipeline.positionMain == "left") {
                        myPosition="left";
                        telemetry.addLine("going left");
                        drive.followTrajectorySequence(scorePurpleLeft);
                    } else if (BluePipeline.positionMain == "middle") {
                        myPosition="middle";
                        telemetry.addLine("going middle");
                        casePos=2;
                        drive.followTrajectorySequence(scorePurpleMiddle);
                    } else {
                        myPosition="right";
                        telemetry.addLine("going right");
                        casePos=3;
                        drive.followTrajectorySequence(scorePurpleRight);
                    }
                    telemetry.update();
                    posEstimate = drive.getPoseEstimate();
                    manip.setIntakePower(-1);
                    sleep(1600);
                    manip.setIntakePower(0);
                    currentState = State.SCORE_YELLOW;
                    break;

                case SCORE_YELLOW:
                    if (BluePipeline.positionMain == "left") {
                        drive.followTrajectorySequence(backDropLeft);
                    } else if (BluePipeline.positionMain == "middle") {
                        drive.followTrajectorySequence(backDropMiddle);
                    } else {
                        drive.followTrajectorySequence(backDropRight);
                    }
                    manip.gateToggle();
                    sleep(500);
                    currentState = State.PARK;
                    break;

                case PARK:
                    if (BluePipeline.positionMain == "left") {
                        drive.followTrajectorySequence(parkLeft);
                    } else if (BluePipeline.positionMain == "middle") {
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
