package org.firstinspires.ftc.teamcode.Autos.RedSide;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.testing.BluePipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "BackdropAutoRed", group = "Autonomous")
public class BackdropAutoRed extends LinearOpMode {

    Manipulators manip;
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
    public static double startPoseX = 14.65845302224215;
    public static double startPoseY = -65.13672263931143;
    public static double startPoseAngle = Math.toRadians(270);

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    // TODO: TUNE ALL POSITIONS

    //going to left spike position
    public static double moveBackwards1 = 31;
    public static double moveForwards1 = 12;
    public static double turn1 = -90;

    //coordinates for left spike position
    public static double spike1X = 6.74633638294297;
    public static double spike1Y = -28.8247700133697;
    public static double spike1Angle = Math.toRadians(180);

    //coordinates for middle spike position
    public static double spike2X = 20;
    public static double spike2Y = -25.08633;
    public static double spike2Angle = Math.toRadians(180);

    //coordinates for right spike position
    public static double spike3X = 28.74633638294297;
    public static double spike3Y = -29.9247700133697;
    public static double spike3Angle = Math.toRadians(180);

    public static double backdropMiddleX = 47;
    public static double backdropMiddleY = -35;
    public static double backdropMiddleAngle = Math.toRadians(180);
    public static double backdropLeftStrafe = 4;
    public static double backdropRightStrafe = 4;

    static double backdropLeftX = 47;
    public static double backdropLeftY = -28;
    public static double backdropLeftAngle = Math.toRadians(180);

    public static double backdropRightX = 47;
    public static double backdropRightY = -41;
    public static double backdropRightAngle = Math.toRadians(180);


    public static double preParkY = -53;
    public static double goingIntoPark = 10;

    public static double temporalMarkerTimeDOWN = .5;
    public static double temporalMarkerTimeUP = 1;


    public static int outtakeEncoderTicks = 2000;
    public static int outtakeOG = 0;

    public static double casenum=1;

    public static BluePipeline.Location positionOfVisionPixel;

    public static String myPosition;

    public static double casePos;

    State currentState = State.IDLE;


    OpenCvWebcam camera;
    public static double color = 1;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        BluePipeline vision =  new BluePipeline(telemetry);
        manip.autoIntakeToggle(false);

        telemetry.addLine("Init Done");

        drive.setPoseEstimate(startPose);
        //

        //still need to enter values for these
        // these are the basic to spike part
        TrajectorySequence scorePurpleLeft = drive.trajectorySequenceBuilder(startPose)
                .back(  25)
                .lineToLinearHeading(new Pose2d(spike1X, spike1Y, spike1Angle))
                .build();

        TrajectorySequence scorePurpleMiddle = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(spike2X, spike2Y, spike2Angle))
                .build();

        TrajectorySequence scorePurpleRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(spike3X, spike3Y, spike3Angle))
                .build();

        TrajectorySequence backDropLeft = drive.trajectorySequenceBuilder(scorePurpleLeft.end())
                .lineToLinearHeading(new Pose2d(backdropLeftX, backdropLeftY, backdropLeftAngle))
                .addTemporalMarker(temporalMarkerTimeUP, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicks);
                })
                .build();

        TrajectorySequence backDropMiddle = drive.trajectorySequenceBuilder(scorePurpleMiddle.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(backdropRightX, backdropRightY, backdropRightAngle))
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
                .strafeLeft(32)
                .turn(Math.toRadians(-90))
                .strafeRight(15)
                .build();

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(backDropMiddle.end())
                .forward(5)
                .addTemporalMarker(temporalMarkerTimeDOWN, () -> {
                    manip.moveOuttakeLift(outtakeOG);
                })
                .strafeLeft(24)
                .turn(Math.toRadians(-90))
                .strafeRight(15)
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(backDropRight.end())
                .forward(5)
                .addTemporalMarker(temporalMarkerTimeDOWN, () -> {
                    manip.moveOuttakeLift(outtakeOG);
                })
                .strafeLeft(18)
                .turn(Math.toRadians(-90))
                .strafeRight(15)
                .build();

        telemetry.addLine("trajectories built!!!");
        telemetry.addData("cpos", vision.getLocation());
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "identifyier","teamcode");
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        BluePipeline detectRed = new BluePipeline(telemetry);
        camera.setPipeline(detectRed);

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
                    currentState = BackdropAutoRed.State.SCORE_PURPLE;

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
                    currentState = BackdropAutoRed.State.SCORE_YELLOW;
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
                    currentState = BackdropAutoRed.State.PARK;
                    break;

                case PARK:
                    if (BluePipeline.positionMain == "left") {
                        drive.followTrajectorySequence(parkLeft);
                    } else if (BluePipeline.positionMain == "middle") {
                        drive.followTrajectorySequence(parkMiddle);
                    } else {
                        drive.followTrajectorySequence(parkRight);
                    }
                    currentState = BackdropAutoRed.State.STOP;
                    break;

                case STOP:
                    break;
            }
        }

    }

}
