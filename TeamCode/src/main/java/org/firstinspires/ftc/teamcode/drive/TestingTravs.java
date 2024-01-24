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
@Autonomous(name = "TestingTrajectories", group = "Autonomous")
public class TestingTravs extends LinearOpMode {


    /* TODO: ENTER COORDINATES FOR ALL POSITIONS
     *   CHANGE ALL VARIABLES TO PUBLIC STATIC
     *   REFACTOR ALL VARIABLES TO MORE DESCRIPTIVE NAMES
     */


    //coordinates for starting position (0, 0, 0)
    public static double startPoseX= 0;
    public static double startPoseY= 0;
    public static double startPoseAngle= 0;

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseAngle));

    Pose2d posEstimate;

    //coordinates for left spike position
    public static double spike1X = -26.854665827113163;
    public static double spike1Y = 47.26695980330197;
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







    OpenCvWebcam camera;
    public static double color = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RedPipeline vision = new RedPipeline(telemetry);


        telemetry.addLine("Init Done");

        drive.setPoseEstimate(startPose);
        //

//        //still need to enter values for these
//        TrajectorySequence scorePurpleLeft = drive.trajectorySequenceBuilder(startPose)
//                .back(42.74444680314227)
//
////                .forward(10)
////                .lineToLinearHeading(new Pose2d(spike1X, spike1Y, spike1Angle))
////                .lineToLinearHeading(new Pose2d(60, -10, 0))
//                .build();

        //still need to enter values for these
        TrajectorySequence scorePurpleMiddle = drive.trajectorySequenceBuilder(startPose)
//                .back(45.74444680314227)
                .lineToLinearHeading(new Pose2d(49.448, 6.499, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(60, -10, 0))

                .build();

        //still need to enter values for these
//        TrajectorySequence scorePurpleRight = drive.trajectorySequenceBuilder(startPose)
////                .forward(10)
////                .lineToLinearHeading(new Pose2d(spike3X, spike3Y, spike3Angle))
////                .lineToLinearHeading(new Pose2d(60, -10, 0))
//                .build();

        telemetry.addLine("trajectories built!!!");

        telemetry.addData("cpos", vision.getLocation());
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "identifyier", "teamcode");
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        RedPipeline detectRed = new RedPipeline(telemetry);
        camera.setPipeline(detectRed);

        camera.setMillisecondsPermissionTimeout(5000);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {


            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        waitForStart();


        camera.stopStreaming();

        while (!isStopRequested() && opModeIsActive()) {
            drive.followTrajectorySequence(scorePurpleMiddle);
            break;

        }
    }}