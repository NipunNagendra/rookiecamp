package org.firstinspires.ftc.teamcode.Autos.RedSide;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "StackAutoRed2plus1Door", group = "Autonomous")
public class StackAutoRed2plus1Door extends LinearOpMode {

    Manipulators manip;
    enum State{
        IDLE,
        SCORE_PURPLE,
        TO_STACK,
        SCORE_YELLOW,
        PARK,
        STOP
    }

    State currentState = State.IDLE;

    /* TODO: ENTER COORDINATES FOR ALL POSITIONS
     *   CHANGE ALL VARIABLES TO PUBLIC STATIC
     *   REFACTOR ALL VARIABLES TO MORE DESCRIPTIVE NAMES
     */


    //coordinates for start position

    public static double startPoseX= -37.89893;
    public static double startPoseY= -62.90750;
    public static double startPoseAngle= Math.toRadians(270);

    public static Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    //coordinates for left spike position
    public static double spike1X = -40.1, spike1Y = -30.2;
    public static double spike1BackX = -34, spike1BackY = spike1Y;
    public static double spike1PreStackX = spike1BackX, spike1PreStackY = -22;

    //coordinates for middle spike position
    public static double spike2X = -38.3, spike2Y = -19.2, spike2Angle = Math.toRadians(330);
    public static double spike2BackX = -45, spike2BackY = -14;
    public static double spike2TurnAmount = Math.toRadians(180) - spike2Angle;

    //coordinates for right spike position
    public static double spike3PreX = -40, spike3PreY = -36;
    public static double spike3X = -30, spike3Y = -30;
    public static double spike3BackAmount = 8;
    public static double spike3PreStackX = -47, spike3PreStackY = -16, spike3PreStackAngle = Math.toRadians(181);

    //coordinates for stack and to backdrop
    public static double doorStackX = -56.3, doorStackY = -12;
    public static double doorStackForwardAmount = BackdropAutoRedSplines2plus2Door.doorStackForwardAmount;
    public static double underDoorX = 24, underDoorY = doorStackY;

    //coordinates for backdrop positions
    public static double backdropMiddleX = 52;
    public static double backdropMiddleY = BackdropAutoRedSplines2plus2Door.backdropMiddleY;
    public static double backdropMiddleAngle = Math.toRadians(180);
    public static double backdropBackAmount = BackdropAutoRedSplines2plus2Door.backdropBackAmount;

    public static double backdropLeftStrafe = BackdropAutoRedSplines2plus2Door.backdropLeftStrafe;
    public static double backdropRightStrafe = BackdropAutoRedSplines2plus2Door.backdropRightStrafe;

    //intake and outtake movement
    public static int outtakeEncoderTicksUp = BackdropAutoRedSplines2plus2Door.outtakeEncoderTicks;
    public static double temporalMarkerTimeUp = 1.5;
    public static int outtakeEncoderTicksDown = 0;
    public static double temporalMarkerTimeDown = 1;

    //coordiantes for alt park
    public static double altParkForwardAmount = 4.5;
    public static double altParkX = 46.5, altParkY = -11;

    public static double casenum = 1;

    public static int myPosition = 1;


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
                .lineToLinearHeading(new Pose2d(spike1X, spike1Y, Math.toRadians(180)))
                .build();

        TrajectorySequence leftSpikeToStack = drive.trajectorySequenceBuilder(new Pose2d(spike1X, spike1Y, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(spike1BackX, spike1BackY))
                .lineToConstantHeading(new Vector2d(spike1PreStackX, spike1PreStackY))
                .waitSeconds(0.01)
                .splineToConstantHeading(new Vector2d(doorStackX, doorStackY), Math.toRadians(180))
                .forward(doorStackForwardAmount)
                .build();

        TrajectorySequence middleToSpike = drive.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(spike2X, spike2Y, spike2Angle))
                .build();

        TrajectorySequence middleSpikeToStack = drive.trajectorySequenceBuilder(middleToSpike.end())
                .lineToConstantHeading(new Vector2d(spike2BackX, spike2BackY))
                .turn(spike2TurnAmount)
                .splineToConstantHeading(new Vector2d(doorStackX, doorStackY), Math.toRadians(180))
                .forward(doorStackForwardAmount)
                .build();

        TrajectorySequence rightToSpike = drive.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .lineToSplineHeading(new Pose2d(spike3PreX, spike3PreY, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(spike3X, spike3Y), Math.toRadians(0))
                .build();

        TrajectorySequence rightSpikeToStack = drive.trajectorySequenceBuilder(new Pose2d(spike3X, spike3Y, Math.toRadians(180)))
                .setReversed(false)
                .back(spike3BackAmount)
                .lineToSplineHeading(new Pose2d(spike3PreStackX, spike3PreStackY, spike3PreStackAngle))
                .waitSeconds(0.01)
                .splineToConstantHeading(new Vector2d(doorStackX, doorStackY), Math.toRadians(180))
                .forward(doorStackForwardAmount)
                .build();

        TrajectorySequence leftStackToBackdrop = drive.trajectorySequenceBuilder(leftSpikeToStack.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(underDoorX, underDoorY))
                .waitSeconds(0.01)
                .splineTo(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY + backdropLeftStrafe), Math.toRadians(0))
                .forward(backdropBackAmount)
                .build();

        TrajectorySequence middleStackToBackdrop = drive.trajectorySequenceBuilder(middleSpikeToStack.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(underDoorX, underDoorY))
                .waitSeconds(0.01)
                .splineTo(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY), Math.toRadians(0))
                .forward(backdropBackAmount)
                .build();

        TrajectorySequence rightStackToBackdrop = drive.trajectorySequenceBuilder(rightSpikeToStack.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(underDoorX, underDoorY))
                .waitSeconds(0.01)
                .splineTo(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY - backdropRightStrafe), Math.toRadians(0))
                .forward(backdropBackAmount)
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(leftStackToBackdrop.end())
                .setReversed(false)
                .forward(altParkForwardAmount)
                .strafeTo(new Vector2d(altParkX, altParkY))
                .build();

        TrajectorySequence middlePark = drive.trajectorySequenceBuilder(middleStackToBackdrop.end())
                .setReversed(false)
                .forward(altParkForwardAmount)
                .strafeTo(new Vector2d(altParkX, altParkY))
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(rightStackToBackdrop.end())
                .setReversed(false)
                .forward(altParkForwardAmount)
                .strafeTo(new Vector2d(altParkX, altParkY))
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
                    currentState = State.TO_STACK;
                    break;

                case TO_STACK:
                    if (myPosition == 1) {
                        drive.followTrajectorySequence(leftSpikeToStack);
                    } else if (myPosition == 2) {
                        drive.followTrajectorySequence(middleSpikeToStack);
                    } else {
                        drive.followTrajectorySequence(rightSpikeToStack);
                    }
                    currentState = State.SCORE_YELLOW;
                    break;

                case SCORE_YELLOW:
                    if (myPosition == 1) {
                        drive.followTrajectorySequence(leftStackToBackdrop);
                    } else if (myPosition == 2) {
                        drive.followTrajectorySequence(middleStackToBackdrop);
                    } else {
                        drive.followTrajectorySequence(rightStackToBackdrop);
                    }
                    manip.gateToggle();
//                    sleep(500);
                    currentState = State.PARK;
                    break;

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
