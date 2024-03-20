package org.firstinspires.ftc.teamcode.Autos.RedSide;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "BackdropAutoRed2plus2Truss", group = "Autonomous")
public class BackdropAutoRedSplines2plus2Truss extends LinearOpMode {

    Manipulators manip;
    enum State{
        IDLE,
        SCORE_PURPLE,
        SCORE_YELLOW,
        CYCLING,
        PARK,
        STOP
    }

    /* TODO: ENTER COORDINATES FOR ALL POSITIONS
     *   CHANGE ALL VARIABLES TO PUBLIC STATIC
     *   REFACTOR ALL VARIABLES TO MORE DESCRIPTIVE NAMES
     */


    //coordinates for start position
    public static double startPoseX = 14.65845302224215;
    public static double startPoseY = -65.13672263931143;
    public static double startPoseAngle = Math.toRadians(270);

    public static Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    //coordinates for left spike position
    public static double spike1ForwardAmount = 5;
    public static double preSpike1X = 19.2;
    public static double preSpike1Y = -37.8;
    public static double preSpike1Angle = Math.toRadians(180);
    public static double spike1X = 8.25;
    public static double spike1Y = -33.8;

    //coordinates for middle spike position
    public static double preSpike2X = 12.3;
    public static double preSpike2Y = -42.3;
    public static double spike2X = 23.8;
    public static double spike2Y = -26;

    //coordinates for right spike position
    public static double spike3X = 29.5;
    public static double spike3Y = -31;
    public static double spike3BackAmount = 3;

    //coordinates for backdrop positions
    public static double backdropMiddleX = 50;
    public static double backdropMiddleY = -35;
    public static double backdropMiddleAngle = Math.toRadians(180);
    public static double backdropBackAmount = 5;

    public static double backdropLeftStrafe = 4;
    public static double backdropRightStrafe = 4;

    //coordinates for cycling
    public static double preTrussX = 20;
    public static double preTrussY = -60.5;
    public static double underTrussX = -47;
    public static double underTrussY = preTrussY;
    public static double postTrussX = -57;
    public static double postTrussY = -46.5;
    public static double trussStackX = -54.3;
    public static double trussStackY = -35.5;
    public static double trussStackForwardAmount = 2;

    //intake and outtake movement
    public static int outtakeEncoderTicksUp = 2500;
    public static double temporalMarkerTimeUp = 1.5;
    public static int outtakeEncoderTicksDown = 0;
    public static double temporalMarkerTimeDown = 1;

    //coordiantes for park in corner
    public static double parkForwardAmount = 3;
    public static double parkStrafeAmount = 15;

    //coordiantes for alt park
    public static double altParkForwardAmount = 4.5;
    public static double preAltParkY = -20;
    public static double preAltParkXChange = -10;
    public static double altParkX = 58.7, altParkY = -12.3;

    public static double casenum = 1;

    public static int myPosition = 1;
    public static double underTrussWaitTime = 0.01;
    public static double parkWaitTime = 0.01;


    public static double temporalMarkerTimeDOWN = .5;
    public static double temporalMarkerTimeUP = 1;


    public static int outtakeEncoderTicks = 2000;
    public static int outtakeOG = 0;

    State currentState = State.IDLE;


    OpenCvWebcam camera;
    public static double color = 1;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
//        BluePipeline vision =  new BluePipeline(telemetry);


        telemetry.addLine("Init Done");

        drive.setPoseEstimate(startPose);
        //

        //still need to enter values for these
        // these are the basic to spike part
        TrajectorySequence leftToSpike = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .back(1)
                .splineToSplineHeading(new Pose2d(preSpike1X, preSpike1Y, preSpike1Angle), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(spike1X, spike1Y), Math.toRadians(0))
                .build();

        TrajectorySequence leftSpikeToBackdrop = drive.trajectorySequenceBuilder(new Pose2d(spike1X, spike1Y, preSpike1Angle))
                .back(spike1ForwardAmount/*,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)*/)

                .splineToConstantHeading(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY + backdropLeftStrafe), Math.toRadians(0))
                .back(backdropBackAmount/*,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)*/)
                .build();

        TrajectorySequence middleToSpike = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(preSpike2X, preSpike2Y), Math.toRadians(90))
                .splineTo(new Vector2d(spike2X, spike2Y), Math.toRadians(0))
                .build();

        TrajectorySequence middleSpikeToBackdrop = drive.trajectorySequenceBuilder(new Pose2d(spike2X, spike2Y, Math.toRadians(180)))
                .setReversed(true)
                .splineTo(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY), Math.toRadians(0))
                .back(backdropBackAmount/*,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)*/)
                .build();

        TrajectorySequence rightToSpike = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(spike3X, spike3Y), Math.toRadians(0))
                .build();

        TrajectorySequence rightSpikeToBackdrop = drive.trajectorySequenceBuilder(new Pose2d(spike3X, spike3Y, Math.toRadians(180)))
                .setReversed(true)
                .back(spike3BackAmount/*,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)*/)
                .waitSeconds(0.5)

                .splineToConstantHeading(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY - backdropRightStrafe), Math.toRadians(0))
                .back(backdropBackAmount/*,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)*/)
                .build();

        TrajectorySequence leftCycle = drive.trajectorySequenceBuilder(new Pose2d(backdropMiddleX, backdropMiddleY + backdropLeftStrafe, Math.toRadians(180)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(preTrussX, preTrussY), Math.toRadians(180))
                .waitSeconds(underTrussWaitTime)
                .lineToConstantHeading(new Vector2d(underTrussX, underTrussY))
                .waitSeconds(underTrussWaitTime)
                .splineToConstantHeading(new Vector2d(postTrussX,  postTrussY), Math.toRadians(90))
                .waitSeconds(underTrussWaitTime)
                .splineToConstantHeading(new Vector2d(trussStackX, trussStackY), Math.toRadians(180))
                .forward(trussStackForwardAmount)
                .build();
        TrajectorySequence leftReturn = drive.trajectorySequenceBuilder(leftCycle.end())
                .setReversed(true)
                .back(trussStackForwardAmount)
                .splineToConstantHeading(new Vector2d(postTrussX, postTrussY), Math.toRadians(270))
                .waitSeconds(underTrussWaitTime)
                .splineToConstantHeading(new Vector2d(underTrussX, underTrussY), Math.toRadians(0))
                .waitSeconds(underTrussWaitTime)
                .lineToConstantHeading(new Vector2d(preTrussX, preTrussY))
                .waitSeconds(underTrussWaitTime)
                .splineToConstantHeading(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY - backdropRightStrafe), Math.toRadians(0))
                .back(backdropBackAmount)
                .build();

        TrajectorySequence middleCycle = drive.trajectorySequenceBuilder(new Pose2d(backdropMiddleX, backdropMiddleY, Math.toRadians(180)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(preTrussX, preTrussY), Math.toRadians(180))
                .waitSeconds(underTrussWaitTime)
                .lineToConstantHeading(new Vector2d(underTrussX, underTrussY))
                .waitSeconds(underTrussWaitTime)
                .splineToConstantHeading(new Vector2d(postTrussX, postTrussY), Math.toRadians(90))
                .waitSeconds(underTrussWaitTime)
                .splineToConstantHeading(new Vector2d(trussStackX, trussStackY), Math.toRadians(180))
                .forward(trussStackForwardAmount)
                .build();
        TrajectorySequence middleReturn = drive.trajectorySequenceBuilder(middleCycle.end())
                .setReversed(true)
                .back(trussStackForwardAmount)
                .splineToConstantHeading(new Vector2d(postTrussX, postTrussY), Math.toRadians(270))
                .waitSeconds(underTrussWaitTime)
                .splineToConstantHeading(new Vector2d(underTrussX, underTrussY), Math.toRadians(0))
                .waitSeconds(underTrussWaitTime)
                .lineToConstantHeading(new Vector2d(preTrussX, preTrussY))
                .waitSeconds(underTrussWaitTime)
                .splineToConstantHeading(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY - backdropRightStrafe), Math.toRadians(0))
                .back(backdropBackAmount)
                .build();


        TrajectorySequence rightCycle = drive.trajectorySequenceBuilder(new Pose2d(backdropMiddleX, backdropMiddleY - backdropRightStrafe, Math.toRadians(180)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(preTrussX, preTrussY), Math.toRadians(180))
                .waitSeconds(underTrussWaitTime)
                .lineToConstantHeading(new Vector2d(underTrussX, underTrussY))
                .waitSeconds(underTrussWaitTime)
                .splineToConstantHeading(new Vector2d(postTrussX, postTrussY), Math.toRadians(90))
                .waitSeconds(underTrussWaitTime)
                .splineToConstantHeading(new Vector2d(trussStackX, trussStackY), Math.toRadians(180))
                .forward(trussStackForwardAmount)
                .build();
        TrajectorySequence rightReturn = drive.trajectorySequenceBuilder(rightCycle.end())
                .setReversed(true)
                .back(trussStackForwardAmount)
                .splineToConstantHeading(new Vector2d(postTrussX, postTrussY), Math.toRadians(270))
                .waitSeconds(underTrussWaitTime)
                .splineToConstantHeading(new Vector2d(underTrussX, underTrussY), Math.toRadians(0))
                .waitSeconds(underTrussWaitTime)
                .lineToConstantHeading(new Vector2d(preTrussX, preTrussY))
                .waitSeconds(underTrussWaitTime)
                .splineToConstantHeading(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY - backdropRightStrafe), Math.toRadians(0))
                .back(backdropBackAmount)
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(new Pose2d(backdropMiddleX, backdropMiddleY - backdropRightStrafe, Math.toRadians(180)))
                .setReversed(false)
                .forward(parkForwardAmount)
                .strafeLeft(parkStrafeAmount)
                .build();

        TrajectorySequence middlePark = drive.trajectorySequenceBuilder(new Pose2d(backdropMiddleX, backdropMiddleY - backdropRightStrafe, Math.toRadians(180)))
                .setReversed(false)
                .forward(parkForwardAmount)
                .strafeLeft(parkStrafeAmount)
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d(backdropMiddleX, backdropMiddleY - backdropRightStrafe, Math.toRadians(180)))
                .setReversed(false)
                .forward(parkForwardAmount)
                .strafeLeft(parkStrafeAmount)
                .build();

        telemetry.addLine("trajectories built!!!");
//        telemetry.addData("cpos", vision.getLocation());
//        telemetry.update();
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "identifyier","teamcode");
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        BluePipeline detectRed = new BluePipeline(telemetry);
//        camera.setPipeline(detectRed);
//
//        camera.setMillisecondsPermissionTimeout(5000);
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//
//                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//
//
//            }
//        });

//        FtcDashboard.getInstance().startCameraStream(camera, 0);

        waitForStart();


//        camera.stopStreaming();

        while(!isStopRequested() && opModeIsActive()){
            posEstimate = drive.getPoseEstimate();

            switch(currentState){
                case IDLE:
                    currentState = State.SCORE_PURPLE;

                case SCORE_PURPLE:
//                    positionOfVisionPixel = vision.getLocation();
                    if (myPosition == 1) {
//                        myPosition="left";
                        telemetry.addLine("going left");
                        drive.followTrajectorySequence(leftToSpike);
                    } else if (myPosition == 2) {
//                        myPosition="middle";
                        telemetry.addLine("going middle");
                        drive.followTrajectorySequence(middleToSpike);
                    } else {
//                        myPosition="right";
                        telemetry.addLine("going right");
                        drive.followTrajectorySequence(rightToSpike);
                    }
                    telemetry.update();
                    posEstimate = drive.getPoseEstimate();
//                    manip.setIntakePower(-1);
//                    sleep(1600);
//                    manip.setIntakePower(0);
                    sleep(500);
                    currentState = State.SCORE_YELLOW;
                    break;

                case SCORE_YELLOW:
                    if (myPosition == 1) {
                        drive.followTrajectorySequence(leftSpikeToBackdrop);
                    } else if (myPosition == 2) {
                        drive.followTrajectorySequence(middleSpikeToBackdrop);
                    } else {
                        drive.followTrajectorySequence(rightSpikeToBackdrop);
                    }
                    manip.gateToggle();
//                    sleep(500);
                    currentState = State.CYCLING;
                    break;

                case CYCLING:
                    if (myPosition == 1) {
                        drive.followTrajectorySequence(leftCycle);
                        drive.followTrajectorySequence(leftReturn);
                    } else if (myPosition == 2) {
                        drive.followTrajectorySequence(middleCycle);
                        drive.followTrajectorySequence(middleReturn);
                    } else {
                        drive.followTrajectorySequence(rightCycle);
                        drive.followTrajectorySequence(rightReturn);
                    }
//                    sleep(500);
                    currentState = State.PARK;

                case PARK:
                    if (myPosition == 1) {
                        drive.followTrajectorySequence(leftPark);
                    } else if (myPosition == 2) {
                        drive.followTrajectorySequence(middlePark);
                    } else {
                        drive.followTrajectorySequence(rightPark);
                    }
                    currentState = State.STOP;
                    break;

                case STOP:
                    break;
            }
        }

    }

}

