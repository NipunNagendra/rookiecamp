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
import org.firstinspires.ftc.teamcode.testing.BluePipeline;
import org.firstinspires.ftc.teamcode.testing.RedPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "StackAutoRed", group = "Autonomous")
public class StackAutoRed extends LinearOpMode {

    enum State{
        IDLE,
        SCORE_PURPLE,
        SCORE_YELLOW,

//        INTAKE_WHITE,
        PARK,
        STOP
    }

    /* TODO: ENTER COORDINATES FOR ALL POSITIONS
    *   CHANGE ALL VARIABLES TO PUBLIC STATIC
    *   REFACTOR ALL VARIABLES TO MORE DESCRIPTIVE NAMES
    */


    //coordinates for starting position (0, 0, 0)
    public static double startPoseX= -38.15845302224215;
    public static double startPoseY= -65.13672263931143;
    public static double startPoseAngle= Math.toRadians(270);

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    //coordinates for left spike position
    public static double spike1X = -41.64633638294297;
    public static double spike1Y = -32.1247700133697;
    public static double spike1Angle = Math.toRadians(180);

    //coordinates for middle spike position
    public static double spike2X =  -37.812297556497846;
    public static double spike2Y = -29.023006373520104;
    public static double spike2Angle = Math.toRadians(90);

    //coordinates for right spike position
//    public static double spike3X = -34.26642694740993;
//    public static double spike3Y = 29.54644728121096;
//    public static double spike3Angle = Math.toRadians(180);
    public static double moveBackwards3 = 31;
    public static double moveForward3 = 10;
    public static double turn3 = 90;

    public static double preTrussX = -38.15845302224215;
    public static double trussX = 40;
    public static double trussY = -60.13672263931143;
    public static double trussAngle = Math.toRadians(180);
    public static double backdropMiddleX = 80;
    public static double backdropMiddleY = -40;
    public static double backdropMiddleAngle = trussAngle;

    public static double casenum=1;

    public static RedPipeline.Location positionOfVisionPixel;

    public static String myPosition;


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
        TrajectorySequence finishLeft = drive.trajectorySequenceBuilder(scorePurpleLeft.end())
                .back(5)
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(preTrussX, trussY, trussAngle))
                .build();
        TrajectorySequence finishMiddle = drive.trajectorySequenceBuilder(scorePurpleMiddle.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(preTrussX, trussY, trussAngle))
                .build();
        TrajectorySequence finishRight = drive.trajectorySequenceBuilder(scorePurpleRight.end())
                .back(10)
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(preTrussX, trussY, trussAngle))
                .build();

        // common trajectory for all 3 paths that leads to the backdrop
        TrajectorySequence underTrussToBackdropAll = drive.trajectorySequenceBuilder(new Pose2d(preTrussX, trussY, trussAngle))
                .lineToLinearHeading(new Pose2d(trussX, trussY, trussAngle))
                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                .build();
        TrajectorySequence strafeToBackdropPosLeft = drive.trajectorySequenceBuilder(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                .strafeLeft(
                        10,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence strafeToBackdropPosRight = drive.trajectorySequenceBuilder(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                .strafeRight(
                        10,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

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
                    sleep(3500);
                    manip.setIntakePower(0);
                    if (myPosition == "left") {
                        drive.followTrajectorySequence(finishLeft);
                    } else if (myPosition == "middle") {
                        drive.followTrajectorySequence(finishMiddle);
                    } else {
                        drive.followTrajectorySequence(finishRight);
                    }
                    currentState = State.SCORE_YELLOW;
                    // ultrasonic + distance sensor stuff here, potentially
                    break;

                case SCORE_YELLOW:
                    drive.followTrajectorySequence(underTrussToBackdropAll);
                    // potentially add sensor stuff here (distance sensor for backdrop)

                    if (myPosition == "left") {
                        drive.followTrajectorySequence(strafeToBackdropPosLeft);
                    } else if (myPosition == "right") {
                        drive.followTrajectorySequence(strafeToBackdropPosRight);
                    } else {
                        // potential placeholder
                    }

                case STOP:
                    break;
            }
        }

    }

}
