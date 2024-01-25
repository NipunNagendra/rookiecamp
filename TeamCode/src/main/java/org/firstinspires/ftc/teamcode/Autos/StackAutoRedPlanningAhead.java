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
    public static double spike1X = -38.64633638294297;
    public static double spike1Y = -32.1247700133697;
    public static double spike1Angle = Math.toRadians(180);

    //coordinates for middle spike position
    public static double spike2X = -20.594631330905656;
    public static double spike2Y = 36.414924287545524;
    public static double spike2Angle = Math.toRadians(180);

    //coordinates for right spike position
    public static double spike3X = -34.26642694740993;
    public static double spike3Y = 29.54644728121096;
    public static double spike3Angle = Math.toRadians(180);

    public static double casenum=1;

    public static RedPipeline.Location positionOfVisionPixel;

    //coordinates for left backdrop position
    public static double backDrop1X = 0;
    public static double backDrop1Y = 0;
    public static double backDrop1Angle = 0;

    //coordinates for middle backdrop position
    public static double backDrop2X = 0;
    public static double backDrop2Y = 0;
    public static double backDrop2Angle = 0;

    // double variable for moving back right before going to the right backdrop
    public static double avoidPixelBack = 0;

    //coordinates for right backdrop position
    public static double backDrop3X = 0;
    public static double backDrop3Y = 0;
    public static double backDrop3Angle = 0;

    //coordinates for door position
    int doorX = 0;
    int doorY = 0;
    int doorAngle = 0;

    //coordinates for white stack position
    int whiteStackX = 0;
    int whiteStackY = 0;
    int whiteStackAngle = 0;

    //coordinates for position right before going closer to the wall for full park
    public static double subParkX = 0;
    public static double subParkY = 0;
    public static double subParkAngle = 0;

    //coordinates for park position
    public static double parkX = 0;
    public static double parkY = 0;
    public static double parkAngle = 0;

    public static double preTruss1X;
    public static double preTruss1Y;
    public static double preTruss1Angle;

    public static double outtakeHeight = 0;


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
        TrajectorySequence scorePurpleLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(spike1X, spike1Y, spike1Angle))
                .build();

        //still need to enter values for these
        TrajectorySequence scorePurpleMiddle = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(spike2X, spike2Y, spike2Angle))
                .build();

        //still need to enter values for these
        TrajectorySequence scorePurpleRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(spike3X, spike3Y, spike3Angle))
                .build();

        TrajectorySequence yellowRightUnderTruss = drive.trajectorySequenceBuilder(scorePurpleLeft.end())
                .lineToLinearHeading(new Pose2d(preTruss1X, preTruss1Y, preTruss1Angle))
                .build();

        TrajectorySequence yellowMiddleUnderTruss = drive.trajectorySequenceBuilder(scorePurpleMiddle.end())
                .lineToLinearHeading(new Pose2d())
                .strafeRight(18)
                .build();

        TrajectorySequence yellowLeftUnderTruss = drive.trajectorySequenceBuilder(scorePurpleRight.end())
                .lineToLinearHeading(startPose)
                .strafeRight(36)
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
                        telemetry.addLine("going left");
                        drive.followTrajectorySequence(scorePurpleLeft);
                    } else if (RedPipeline.positionMain == "middle") {
                        telemetry.addLine("going middle");
                        drive.followTrajectorySequence(scorePurpleMiddle);
                    } else {
                        telemetry.addLine("going right");
                        drive.followTrajectorySequence(scorePurpleRight);
                    }
                    telemetry.update();
                    manip.setIntakePower(-0.6);
                    sleep(3500);
                    manip.setIntakePower(0);
                    currentState = State.STOP;
                    break;

                case STOP:
                    break;
            }
        }

    }

}
