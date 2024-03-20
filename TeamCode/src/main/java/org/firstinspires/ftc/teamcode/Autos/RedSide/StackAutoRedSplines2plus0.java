package org.firstinspires.ftc.teamcode.Autos.RedSide;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.libs.SensorLibrary;
import org.firstinspires.ftc.teamcode.testing.RedPipeline;
import org.firstinspires.ftc.teamcode.testing.BluePipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "StackAutoRedSplines2+0", group = "Autonomous")
public class StackAutoRedSplines2plus0 extends LinearOpMode {

    Manipulators manip;
    enum State{
        IDLE,
        SCORE_PURPLE,
//        INTAKE_STACK,
        UNDER_DOOR_OR_TRUSS,
        SCORE_YELLOW,
        PARK,
        STOP
    }

    /* TODO: ENTER COORDINATES FOR ALL POSITIONS
     *   CHANGE ALL VARIABLES TO PUBLIC STATIC
     *   REFACTOR ALL VARIABLES TO MORE DESCRIPTIVE NAMES
     */


    //coordinates for starting position (0, 0, 0)
    public static double startPoseX= -37.95845302224215;
    public static double startPoseY= -65.63672263931143;
    public static double startPoseAngle= Math.toRadians(270);

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    //coordinates for left spike position
    public static double spike1X = -39.64633638294297;
    public static double spike1Y = -32.1247700133697;
    public static double spike1Angle = Math.toRadians(180);
    public static double spike1Strafe = 10;
    public static double spike1BackAmount = 3;
    public static double spike1StackOffset = 5;

    //coordinates for middle spike position
    public static double spike2X =  -37.812297556497846;
    public static double spike2Y = -32.023006373520104;
    public static double spike2Angle = Math.toRadians(90);

    //coordinates for right spike position
    public static double spike3X = -32.76642694740993;
    public static double spike3Y = -34.04644728121096;
    public static double spike3Angle = Math.toRadians(0);

//    public static double moveBackwards3 = 31;
//    public static double moveForward3 = 11;
//    public static double turn3 = 90;

    public static double firstStackX = -60;
    public static double firstStackY = -35;
    public static double firstStackAngle = Math.toRadians(180);

    public static double prePreTrussX = -55.7;
    public static double prePreTrussY = -39;
    public static double preTrussX = -32.5;
    public static double trussX = 27;
    public static double trussY = -60;
    public static double trussAngle = Math.toRadians(180);

    public static double preBackdropX = 42;
    public static double preBackdropY = -35.30;

    public static double backdropMiddleX = 54;
    public static double backdropMiddleY = -35.3;
    public static double backdropMiddleAngle = trussAngle;
    public static double backdropLeftStrafe = 8;
    public static double backdropRightStrafe = 8;

    public static double preParkX = 46.5;
    public static double preParkY = -61;

    public static double moveBackwards3 = 29;
    public static double moveForward3 = 10;
    public static double turn3 = 90;
    public static double dangerPathX = -15;

    public static double goingDirectlyUnderTruss = 15;
    public static double betweenTruss = 35;
    public static double exitDoor = 15;

    public static double goingIntoPark = 18;

    public static double temporalMarkerTimeAlternate = 4;

    public static double outFromBackdrop = 10;

    public static int outtakeEncoderTicksUp = 1500;
    public static int outtakeEncoderTicksDown = 0;
    public static double temporalMarkerTimeUp = 5;
    public static double temporalMarkerTimeDown = 0.5;

    public static double casenum=1;

    public static RedPipeline.Location positionOfVisionPixel;

    public static String myPosition;
    public static Boolean danger = Boolean.FALSE;

    SensorLibrary sLib;


    State currentState = State.IDLE;
    public static double xyParameter = .2;
    public static double headingParameter = .2;

    public static ElapsedTime lockTime = new ElapsedTime();

    double lockThreshold = 0;
    Pose2d lockLocation = new Pose2d(0, 0, 0);

    OpenCvWebcam camera;
    public static double color = 1;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        RedPipeline vision =  new RedPipeline(telemetry);



        telemetry.addLine("Init Done");
        manip.autoIntakeToggle(false);
        drive.setPoseEstimate(startPose);
        //

        //still need to enter values for these
        // these are the basic to spike part
        TrajectorySequence scorePurpleLeft = drive.trajectorySequenceBuilder(startPose)
//                .setReversed(true)
                .lineToLinearHeading(new Pose2d(spike1X, spike1Y, spike1Angle))
                .build();

        TrajectorySequence scorePurpleMiddle = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(spike2X, spike2Y, spike2Angle))
                .build();

        TrajectorySequence scorePurpleRight = drive.trajectorySequenceBuilder(startPose)
                .back(moveBackwards3)
                .turn(Math.toRadians(turn3))
                .forward(moveForward3)
                .build();

        // post-spike, moving towards prev truss
//        TrajectorySequence finishLeft = drive.trajectorySequenceBuilder(scorePurpleLeft.end())
//                .back(5)
////                .turn(Math.toRadians(90))
//                .lineToLinearHeading(
//                        new Pose2d(preTrussX, trussY, trussAngle),
//                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//        TrajectorySequence finishMiddle = drive.trajectorySequenceBuilder(scorePurpleMiddle.end())
//                .back(5)
//                .lineToLinearHeading(new Pose2d(preTrussX, trussY, trussAngle))
//                .build();
//        TrajectorySequence finishRight = drive.trajectorySequenceBuilder(scorePurpleRight.end())
//                .back(10)
//                .turn(Math.toRadians( -180))
//                .lineToLinearHeading(
//                        new Pose2d(preTrussX, trussY, trussAngle),
//                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();

//        TrajectorySequence goToStackLeft = drive.trajectorySequenceBuilder(scorePurpleLeft.end())
//                .back(8)
//                .turn(Math.toRadians(60))
//                .setReversed(false)
//                .splineTo/*SplineHeading*/(new Vector2d(firstStackX + 1, firstStackY), Math.toRadians(180))
//                .forward(1)
//                .build();
//
//        TrajectorySequence goToStackMiddle = drive.trajectorySequenceBuilder(scorePurpleMiddle.end())
//                .splineToConstantHeading(new Vector2d(spike2X, -37), Math.toRadians(180))
//                .splineToSplineHeading(new Pose2d(firstStackX + 1, firstStackY, firstStackAngle), Math.toRadians(180))
//                .forward(1)
//                .setReversed(false)
//                .build();
//
//        TrajectorySequence goToStackRight = drive.trajectorySequenceBuilder(scorePurpleRight.end())
//                .back(2)
//                .splineToSplineHeading(new Pose2d(firstStackX + 3, firstStackY, firstStackAngle), Math.toRadians(180))
//                .forward(4)
//                .setReversed(false)
//                .build();

        TrajectorySequence goToBackdropLeft = drive.trajectorySequenceBuilder(scorePurpleLeft.end())
                .addTemporalMarker(temporalMarkerTimeUp, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksUp);
                })
                .setReversed(true)
                .back(3)
                .lineToLinearHeading(new Pose2d(preTrussX, trussY, trussAngle),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .lineToLinearHeading(new Pose2d(preTrussX + 1, trussY, trussAngle))
                .lineTo(new Vector2d(trussX, trussY),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(preBackdropX, preBackdropY), Math.toRadians(0))

                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY + backdropLeftStrafe, backdropMiddleAngle),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence goToBackdropMiddle = drive.trajectorySequenceBuilder(scorePurpleMiddle.end())
                .addTemporalMarker(temporalMarkerTimeUp, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksUp);
                })
                // to backdrop
                .back(5)
                .lineToLinearHeading(new Pose2d(preTrussX, trussY, trussAngle))
                .lineTo(new Vector2d(trussX, trussY),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(preBackdropX, preBackdropY), Math.toRadians(0))

                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence goToBackdropRight = drive.trajectorySequenceBuilder(scorePurpleRight.end())
                .addTemporalMarker(temporalMarkerTimeUp, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksUp);
                })
                .setReversed(true)
                .back(10)
                .lineToLinearHeading(new Pose2d(preTrussX, trussY, trussAngle))
                .lineTo(new Vector2d(trussX, trussY),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(preBackdropX, preBackdropY), Math.toRadians(0))

                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY - backdropRightStrafe, backdropMiddleAngle),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(goToBackdropLeft.end())
                .addTemporalMarker(temporalMarkerTimeDown, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksDown);
                })
                .forward(3)
                .splineToConstantHeading(new Vector2d(preParkX, preParkY), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(preParkX + 1, preParkY, Math.toRadians(90)), Math.toRadians(0))
                .turn(Math.toRadians(-90))
                .strafeRight(10)
                .build();

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(goToBackdropMiddle.end())
                .addTemporalMarker(temporalMarkerTimeDown, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksDown);
                })
                .forward(3)
                .splineToConstantHeading(new Vector2d(preParkX, preParkY), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(preParkX + 1, preParkY, Math.toRadians(90)), Math.toRadians(0))
                .turn(Math.toRadians(-90))
                .strafeRight(10)
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(goToBackdropRight.end())
                .addTemporalMarker(temporalMarkerTimeDown, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksDown);
                })
                .forward(3)
                .splineToConstantHeading(new Vector2d(preParkX, preParkY), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(preParkX + 1, preParkY, Math.toRadians(90)), Math.toRadians(0))
                .turn(Math.toRadians(-90))
                .strafeRight(10)
                .build();

        TrajectorySequence underDoorScuffed = drive.trajectorySequenceBuilder(new Pose2d(preTrussX, trussY, trussAngle))
                .addTemporalMarker(temporalMarkerTimeAlternate, () -> {
                    manip.moveOuttakeLift(outtakeEncoderTicksUp);
                })
                .lineToLinearHeading(new Pose2d(dangerPathX, trussY, trussAngle))
                .strafeRight(
                        betweenTruss,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle), Math.toRadians(270))
                .build();

        // common trajectory for all 3 paths that leads to the backdrop
//        TrajectorySequence underTrussToBackdropAll = drive.trajectorySequenceBuilder(goToStackLeft.end())
//                .addTemporalMarker(temporalMarkerTimeUp, () -> {
//                    manip.moveOuttakeLift(outtakeEncoderTicksUp);
//                })
//                .setReversed(true)
//                .splineTo(new Vector2d(prePreTrussX, prePreTrussY), Math.toRadians(0))
//                .splineTo(new Vector2d(preTrussX, trussY), Math.toRadians(0))
//                .splineTo(new Vector2d(trussX, trussY), Math.toRadians(0))
//                .splineTo(new Vector2d(preBackdropX, preBackdropY), Math.toRadians(0))
//                .lineToLinearHeading(
//                        new Pose2d(backdropMiddleX, backdropMiddleY + backdropLeftStrafe, backdropMiddleAngle), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                )
//                .build();
//        TrajectorySequence strafeToBackdropPosLeft = drive.trajectorySequenceBuilder(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
//                .strafeRight(
//                        backdropLeftStrafe,
//                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//        TrajectorySequence strafeToBackdropPosRight = drive.trajectorySequenceBuilder(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
//                .strafeLeft(
//                        backdropRightStrafe,
//                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//        TrajectorySequence parkAll = drive.trajectorySequenceBuilder(underTrussToBackdropAll.end())
//                .forward(3)
//                .splineToConstantHeading(new Vector2d(preParkX, preParkY), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(preParkX + 1, preParkY, Math.toRadians(90)), Math.toRadians(0))
//                .strafeRight(10)
//                .build();
//        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(strafeToBackdropPosLeft.end())
////                .addTemporalMarker(temporalMarkerTimeDown, () -> {
////                    manip.moveOuttakeLift(outtakeEncoderTicksDown);
////                })
//                .forward(outFromBackdrop)
//                .lineToLinearHeading(new Pose2d(backdropMiddleX - outFromBackdrop, preParkY, startPoseAngle - Math.toRadians(90)))
//                .turn(Math.toRadians(-90))
//                .strafeRight(goingIntoPark)
//                .build();
//        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(underTrussToBackdropAll.end())
////                .addTemporalMarker(temporalMarkerTimeDown, () -> {
////                    manip.moveOuttakeLift(outtakeEncoderTicksDown);
////                })
//                .forward(outFromBackdrop)
//                .lineToLinearHeading(new Pose2d(backdropMiddleX - outFromBackdrop, preParkY, startPoseAngle - Math.toRadians(90)))
//                .turn(Math.toRadians(-90))
//                .strafeRight(goingIntoPark)
//                .build();
//        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(strafeToBackdropPosRight.end())
////                .addTemporalMarker(temporalMarkerTimeDown, () -> {
////                    manip.moveOuttakeLift(outtakeEncoderTicksDown);
////                })
//                .forward(outFromBackdrop)
//                .lineToLinearHeading(new Pose2d(backdropMiddleX - outFromBackdrop, preParkY, startPoseAngle - Math.toRadians(90)))
//                .turn(Math.toRadians(-90))
//                .strafeRight(goingIntoPark)
//                .build();

        telemetry.addLine("trajectories built!!!");

        telemetry.addData("cpos", vision.getLocation());
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "identifyier","teamcode");
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        RedPipeline detectRed = new RedPipeline(telemetry);
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

            switch(currentState){
                case IDLE:
//                    manip.moveOuttakeLift(5000);
                    currentState = State.SCORE_PURPLE;

                case SCORE_PURPLE:
                    manip.autoIntakeToggle(false);
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
                    manip.setIntakePower(-1);
                    sleep(1500);
                    manip.setIntakePower(0);
//                    if (myPosition == "left") {
//                        drive.followTrajectorySequence(finishLeft);
//                    } else if (myPosition == "middle") {
//                        drive.followTrajectorySequence(finishMiddle);
//                    } else {
//                        drive.followTrajectorySequence(finishRight);
//                    }

                    currentState = State.UNDER_DOOR_OR_TRUSS;

//                case INTAKE_STACK:
//                    if (myPosition == "left") {
//                        drive.followTrajectorySequence(goToStackLeft);
//                        lockLocation = goToStackLeft.end();
//                    } else if (myPosition == "middle") {
//                        drive.followTrajectorySequence(goToStackMiddle);
//                        lockLocation = goToStackMiddle.end();
//                    } else {
//                        drive.followTrajectorySequence(goToStackRight);
//                        lockLocation = goToStackMiddle.end();
//                    }
//                    lockTime.reset();
////                    while(lockTime.seconds()<=2) {
////                        lockTo(lockLocation, drive);
////                    }
//                    currentState = State.UNDER_DOOR_OR_TRUSS;

                case UNDER_DOOR_OR_TRUSS:
                    if (myPosition == "left") {
                        drive.followTrajectorySequence(goToBackdropLeft);
                    } else if (myPosition == "middle") {
                        drive.followTrajectorySequence(goToBackdropMiddle);
                    } else {
                        drive.followTrajectorySequence(goToBackdropRight);
                    }

                    currentState = State.SCORE_YELLOW;

                case SCORE_YELLOW:
//                    if (myPosition == "left") {
//                        posEstimate = new Pose2d(backdropMiddleX, backdropMiddleY + backdropLeftStrafe, backdropMiddleAngle);
//                        drive.followTrajectorySequence(strafeToBackdropPosLeft);
//                    } else if (myPosition == "right") {
//                        posEstimate = new Pose2d(backdropMiddleX, backdropMiddleY - backdropRightStrafe, backdropMiddleAngle);
//                        drive.followTrajectorySequence(strafeToBackdropPosRight);
//                    } else {
//                        posEstimate = new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle);
//                    }
                    manip.gateToggle();
                    manip.gateToggle();
                    sleep(1500);
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

    public void lockTo(Pose2d targetPos, SampleMecanumDrive drive) {
        Pose2d currPos = drive.getPoseEstimate();
        Pose2d difference = targetPos.minus(currPos);
        Vector2d xy = difference.vec().rotated(-currPos.getHeading());
        double heading = Angle.normDelta(targetPos.getHeading()) - Angle.normDelta(currPos.getHeading());

        drive.setWeightedDrivePower(new Pose2d(xy.times(xyParameter), heading* headingParameter));
    }

}
