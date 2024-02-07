package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.libs.SensorLibrary;
import org.firstinspires.ftc.teamcode.testing.RedPipeline;
import org.firstinspires.ftc.teamcode.testing.RedPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "StackAutoBlue", group = "Autonomous")
public class StackAutoBlue extends LinearOpMode {

    Manipulators manip;
    enum State{
        IDLE,
        SCORE_PURPLE,
        UNDER_DOOR_OR_TRUSS,
        SCORE_YELLOW,
        PARK,
        STOP
    }

    /* TODO: ENTER COORDINATES FOR ALL POSITIONS
     *   CHANGE ALL VARIABLES TO PUBLIC STATIC
     *   REFACTOR ALL VARIABLES TO MORE DESCRIPTIVE NAMES
     */


    //coordinates for starting position
    public static double startPoseX = -38.15845302224215;
    public static double startPoseY = 65.13672263931143;
    public static double startPoseAngle = Math.toRadians(90);

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;


    //going to left spike position
    public static double moveBackwards1 = 31;
    public static double moveForwards1 = 12;
    public static double turn1 = -90;

    //coordinates for middle spike position
    public static double spike2X = -37.812297556497846;
    public static double spike2Y = 27.023006373520104;
    public static double spike2Angle = Math.toRadians(270);

    //coordinates for right spike position
    public static double spike3X = -41.64633638294297;
    public static double spike3Y = 32.1247700133697;
    public static double spike3Angle = Math.toRadians(180);

    public static double preTrussX = -38.15845302224215;
    public static double trussX = 20;
    public static double trussY = 57.93672263931143;
    public static double trussAngle = Math.toRadians(180);

    public static double goingDirectlyUnderTruss = 26;
    public static double betweenTruss = 46;
    public static double exitDoor = 45;

    public static double backdropMiddleX = 52;
    public static double backdropMiddleY = 35;
    public static double backdropMiddleAngle = trussAngle;
    public static double backdropLeftStrafe = 6;
    public static double backdropRightStrafe = 6;

    public static double outFromBackdrop = 10;
    public static double preParkY = 58.5;
    public static double goingIntoPark = 15;

    public static int outtakeEncoderTicksUp = 2500;
    public static int outtakeEncoderTicksDown = 0;
    public static double temporalMarkerTimeUp = 1.5;
    public static double temporalMarkerTimeDown = 0.5;

    public static double casenum=1;

    public static RedPipeline.Location positionOfVisionPixel;

    public static String myPosition;
    public static Boolean danger = Boolean.FALSE;

    SensorLibrary sLib;


    State currentState = State.IDLE;


    OpenCvWebcam camera;
    public static double color = 1;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        RedPipeline vision =  new RedPipeline(telemetry);


        telemetry.addLine("Init Done");

        drive.setPoseEstimate(startPose);
        //

        //still need to enter values for these
        // these are the basic to spike part
        TrajectorySequence scorePurpleLeft = drive.trajectorySequenceBuilder(startPose)
                .back(moveBackwards1)
                .turn(Math.toRadians(turn1))
                .forward(moveForwards1)
                .build();

        TrajectorySequence scorePurpleMiddle = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(spike2X, spike2Y, spike2Angle))
                .build();

        TrajectorySequence scorePurpleRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(spike3X, spike3Y, spike3Angle))
                .build();

        // post-spike, moving towards prev truss
        TrajectorySequence finishLeft = drive.trajectorySequenceBuilder(scorePurpleLeft.end())
                .back(moveForwards1)
                .turn(Math.toRadians(-180))
                .lineToLinearHeading(
                        new Pose2d(preTrussX, trussY, trussAngle),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence finishMiddle = drive.trajectorySequenceBuilder(scorePurpleMiddle.end())
                .back(5)
                .lineToLinearHeading(
                        new Pose2d(preTrussX, trussY, trussAngle),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence finishRight = drive.trajectorySequenceBuilder(scorePurpleRight.end())
                .back(startPoseX - spike3X)
                .lineToLinearHeading(
                        new Pose2d(preTrussX, trussY, trussAngle),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence underDoorScuffed = drive.trajectorySequenceBuilder(new Pose2d(preTrussX, trussY, trussAngle))
                .addTemporalMarker(temporalMarkerTimeUp, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksUp);
                })
                .back(goingDirectlyUnderTruss)
                .strafeRight(
                        betweenTruss,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(exitDoor)
                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                .build();

        // common trajectory for all 3 paths that leads to the backdrop
        TrajectorySequence underTrussToBackdropAll = drive.trajectorySequenceBuilder(new Pose2d(preTrussX, trussY, trussAngle))
                .addTemporalMarker(temporalMarkerTimeUp, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksUp);
                })
                .lineToLinearHeading(new Pose2d(trussX, trussY, trussAngle))
                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                .build();
        TrajectorySequence strafeToBackdropPosLeft = drive.trajectorySequenceBuilder(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                .strafeRight(
                        backdropLeftStrafe,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence strafeToBackdropPosRight = drive.trajectorySequenceBuilder(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                .strafeLeft(
                        backdropRightStrafe,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(strafeToBackdropPosLeft.end())
                .addTemporalMarker(temporalMarkerTimeDown, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksDown);
                })
                .forward(outFromBackdrop)
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(backdropMiddleX - outFromBackdrop, preParkY, startPoseAngle - Math.toRadians(180)))
                .strafeLeft(goingIntoPark)
                .build();
        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(underTrussToBackdropAll.end())
                .addTemporalMarker(temporalMarkerTimeDown, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksDown);
                })
                .forward(outFromBackdrop)
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(backdropMiddleX - outFromBackdrop, preParkY, startPoseAngle - Math.toRadians(180)))
                .strafeLeft(goingIntoPark)
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(strafeToBackdropPosRight.end())
                .addTemporalMarker(temporalMarkerTimeDown, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksDown);
                })
                .forward(outFromBackdrop)
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(backdropMiddleX - outFromBackdrop, preParkY, startPoseAngle - Math.toRadians(180)))
                .strafeLeft(goingIntoPark)
                .build();

        telemetry.addLine("trajectories built!!!");

        telemetry.addData("cpos", vision.getLocation());
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "identifyier","teamcode");
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        RedPipeline detectBlue = new RedPipeline(telemetry);
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

            switch(currentState){
                case IDLE:
//                    manip.moveOuttakeLift(5000);
                    currentState = State.SCORE_PURPLE;

                case SCORE_PURPLE:
                    if (RedPipeline.positionMain == "left") {
                        myPosition="left";
                        telemetry.addLine("going left");
                        drive.followTrajectorySequence(scorePurpleLeft);
                    } else if (RedPipeline.positionMain == "middle") {
                        myPosition="middle";
                        telemetry.addLine("going middle");
                        drive.followTrajectorySequence(scorePurpleMiddle);
                    } else {
                        myPosition="right";
                        telemetry.addLine("going right");
                        drive.followTrajectorySequence(scorePurpleRight);
                    }
                    telemetry.update();
                    manip.setIntakePower(-0.6);
                    sleep(1000);
                    manip.setIntakePower(0);
                    if (myPosition == "left") {
                        drive.followTrajectorySequence(finishLeft);
                    } else if (myPosition == "middle") {
                        drive.followTrajectorySequence(finishMiddle);
                    } else {
                        drive.followTrajectorySequence(finishRight);
                    }

                    currentState = State.UNDER_DOOR_OR_TRUSS;

                case UNDER_DOOR_OR_TRUSS:
                    // ultrasonic + distance sensor stuff here idk
                    if (/* distance sensor detects robot block */ danger) {
                        drive.followTrajectorySequence(underDoorScuffed);
                    }
                    else {
                        drive.followTrajectorySequence(underTrussToBackdropAll);
                    }
                    currentState = State.SCORE_YELLOW;

                case SCORE_YELLOW:

                    if (myPosition == "left") {
                        posEstimate = new Pose2d(backdropMiddleX, backdropMiddleY + backdropLeftStrafe, backdropMiddleAngle);
                        drive.followTrajectorySequence(strafeToBackdropPosLeft);
                    } else if (myPosition == "right") {
                        posEstimate = new Pose2d(backdropMiddleX, backdropMiddleY - backdropRightStrafe, backdropMiddleAngle);
                        drive.followTrajectorySequence(strafeToBackdropPosRight);
                    } else {
                        posEstimate = new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle);
                    }
                    sleep(500);
                    manip.gateToggle();
                    sleep(500);
                    currentState = State.PARK;

                case PARK:
                    if (myPosition == "left") {
                        drive.followTrajectorySequence(parkLeft);
                    } else if (myPosition == "right") {
                        drive.followTrajectorySequence(parkRight);
                    } else {
                        drive.followTrajectorySequence(parkMiddle);
                    }
                    currentState = State.STOP;

                case STOP:
                    break;
            }
        }

    }

}

