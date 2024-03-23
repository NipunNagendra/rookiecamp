package org.firstinspires.ftc.teamcode.Autos.BlueSide;

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
@Autonomous(name = "BackdropAutoBlueSplines2plus2Door", group = "Autonomous")
public class BackdropAutoBlueSplines2plus2Door extends LinearOpMode {

    Manipulators manip;
    enum State{
        IDLE,
        SCORE_PURPLE,
        SCORE_YELLOW,
        CYCLING,
        PARK,
        STOP
    }

    State currentState = State.IDLE;

    /* TODO: ENTER COORDINATES FOR ALL POSITIONS
     *   CHANGE ALL VARIABLES TO PUBLIC STATIC
     *   REFACTOR ALL VARIABLES TO MORE DESCRIPTIVE NAMES
     */


    //coordinates for start position

    public static double startPoseX = 14.65845302224215;
    public static double startPoseY = 65.13672263931143;
    public static double startPoseAngle = Math.toRadians(270);

    public static Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    //coordinates for left spike position
    public static double spike1X = 29.5;
    public static double spike1Y = 31;
    public static double spike1BackAmount = 3;

    //coordinates for middle spike position
    public static double preSpike2X = 12.3;
    public static double preSpike2Y = 42.3;
    public static double spike2X = 23.8;
    public static double spike2Y = 26;

    //coordinates for right spike position
    public static double spike3ForwardAmount = 5;
    public static double preSpike3X = 19.2;
    public static double preSpike3Y = 37.8;
    public static double preSpike3Angle = Math.toRadians(180);
    public static double spike3X = 8.25;
    public static double spike3Y = 33.8;


    //coordinates for backdrop positions
    public static double backdropMiddleX = 50;
    public static double backdropMiddleY = 35;
    public static double backdropMiddleAngle = Math.toRadians(180);
    public static double backdropBackAmount = 5;

    public static double backdropLeftStrafe = 4;
    public static double backdropRightStrafe = 4;

    //coordinates for cycling
    public static double preDoorX = 24;
    public static double preDoorY = 12;
    public static double underDoorX = -54.3;
    public static double underDoorY = 13;
    public static double doorStackForwardAmount = 2;
    public static double backUnderDoorX = 27;

    //intake and outtake movement
    public static int outtakeEncoderTicksUp = 2500;
    public static double temporalMarkerTimeUp = 1.5;
    public static int outtakeEncoderTicksDown = 0;
    public static double temporalMarkerTimeDown = 1;
    public static int outtakeEncoderTicksCycle = 3000;
    public static double temporalMarkerTimeCycleUp = 4;
    public static double temporalMarkerTimeCycleDown = 0.5;

    //coordiantes for alt park
    public static double altParkForwardAmount = 4.5;
    public static double altParkX = 58.7, altParkY = 12.3;

    public static double casenum = 1;

    public static int myPosition = 1;
    public static double underDoorWaitTime = 0.01;
    public static double altParkWaitTime = 0.01;


    public static double temporalMarkerTimeDOWN = .5;
    public static double temporalMarkerTimeUP = 1;


    public static int outtakeEncoderTicks = 2000;
    public static int outtakeOG = 0;

//    public static BluePipeline.Location positionOfVisionPixel;

    public static double casePos;


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
                .splineTo(new Vector2d(spike1X, spike1Y), Math.toRadians(0))
                .build();

        TrajectorySequence leftSpikeToBackdrop = drive.trajectorySequenceBuilder(leftToSpike.end())
                .addTemporalMarker(temporalMarkerTimeUp, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksUp);
                })
                .setReversed(true)
                .back(spike1BackAmount/*,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)*/)
                .waitSeconds(0.01)

                .splineToConstantHeading(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY + backdropLeftStrafe), Math.toRadians(0))
                .back(backdropBackAmount,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence middleToSpike = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(preSpike2X, preSpike2Y), Math.toRadians(90))
                .splineTo(new Vector2d(spike2X, spike2Y), Math.toRadians(0))
                .build();

        TrajectorySequence middleSpikeToBackdrop = drive.trajectorySequenceBuilder(new Pose2d(spike2X, spike2Y, Math.toRadians(180)))
                .addTemporalMarker(temporalMarkerTimeUp, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksUp);
                })
                .setReversed(true)
                .splineTo(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY), Math.toRadians(0))
                .back(backdropBackAmount,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence rightToSpike = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .back(1)
                .splineToSplineHeading(new Pose2d(preSpike3X, preSpike3Y, preSpike3Angle), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(spike3X, spike3Y), Math.toRadians(0))
                .build();

        TrajectorySequence rightSpikeToBackdrop = drive.trajectorySequenceBuilder(rightToSpike.end())
                .addTemporalMarker(temporalMarkerTimeUp, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksUp);
                })
                .back(spike3ForwardAmount/*,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)*/)

                .splineToConstantHeading(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY - backdropRightStrafe), Math.toRadians(0))
                .back(backdropBackAmount,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence leftCycle = drive.trajectorySequenceBuilder(leftSpikeToBackdrop.end())
                .addTemporalMarker(temporalMarkerTimeDown, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksDown);
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(preDoorX, preDoorY), Math.toRadians(180))
                .waitSeconds(underDoorWaitTime)
                .lineToConstantHeading(new Vector2d(underDoorX, underDoorY))
                .forward(doorStackForwardAmount,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence leftReturn = drive.trajectorySequenceBuilder(leftCycle.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(backUnderDoorX, preDoorY))
                .splineToConstantHeading(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY - backdropRightStrafe), Math.toRadians(0))
                .back(backdropBackAmount,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence middleCycle = drive.trajectorySequenceBuilder(new Pose2d(backdropMiddleX, backdropMiddleY, Math.toRadians(180)))
                .addTemporalMarker(temporalMarkerTimeDown, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksDown);
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(preDoorX, preDoorY), Math.toRadians(180))
                .waitSeconds(underDoorWaitTime)
                .splineToConstantHeading(new Vector2d(underDoorX, underDoorY), Math.toRadians(180))
                .forward(doorStackForwardAmount,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence middleReturn = drive.trajectorySequenceBuilder(middleCycle.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(backUnderDoorX, preDoorY))
                .splineToConstantHeading(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY + backdropRightStrafe), Math.toRadians(0))
                .back(backdropBackAmount,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence rightCycle = drive.trajectorySequenceBuilder(new Pose2d(backdropMiddleX, backdropMiddleY - backdropRightStrafe, Math.toRadians(180)))
                .addTemporalMarker(temporalMarkerTimeDown, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksDown);
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(preDoorX, preDoorY), Math.toRadians(180))
                .waitSeconds(underDoorWaitTime)
                .splineToConstantHeading(new Vector2d(underDoorX, underDoorY), Math.toRadians(180))
                .forward(doorStackForwardAmount,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence rightReturn = drive.trajectorySequenceBuilder(rightCycle.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(backUnderDoorX, preDoorY))
                .splineToConstantHeading(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY), Math.toRadians(0))
                .back(backdropBackAmount,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(leftReturn.end())
                .setReversed(false)
                .forward(altParkForwardAmount)
                .strafeTo(new Vector2d(altParkX, altParkY))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence middlePark = drive.trajectorySequenceBuilder(middleReturn.end())
                .setReversed(false)
                .forward(altParkForwardAmount)
                .strafeTo(new Vector2d(altParkX, altParkY))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(rightReturn.end())
                .setReversed(false)
                .forward(altParkForwardAmount)
                .strafeTo(new Vector2d(altParkX, altParkY))
                .turn(Math.toRadians(90))
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
                    manip.autoIntakeToggle(true);
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
                    manip.setIntakePower(-1);
                    sleep(1500);
                    manip.setIntakePower(0);
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
                        manip.autoIntakeToggle(false);
                        manip.setIntakePower(1);
                        sleep(2500);
                        manip.autoIntakeToggle(true);
                        sleep(500);
                        drive.followTrajectorySequence(leftReturn);
                    } else if (myPosition == 2) {
                        drive.followTrajectorySequence(middleCycle);
                        manip.autoIntakeToggle(false);
                        manip.setIntakePower(1);
                        sleep(2500);
                        manip.autoIntakeToggle(true);
                        sleep(500);
                        drive.followTrajectorySequence(middleReturn);
                    } else {
                        drive.followTrajectorySequence(rightCycle);
                        manip.autoIntakeToggle(false);
                        manip.setIntakePower(1);
                        sleep(2500);
                        manip.autoIntakeToggle(true);
                        sleep(500);
                        drive.followTrajectorySequence(rightReturn);
                    }
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