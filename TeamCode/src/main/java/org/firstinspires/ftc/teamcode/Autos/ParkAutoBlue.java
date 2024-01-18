package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.testing.RedPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "ParkAutoBlue", group = "Autonomous")
public class ParkAutoBlue extends LinearOpMode {

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
    public static double startPoseX= -60.2817198;
    public static double startPoseY= -11.20199806;
    public static double startPoseAngle= 0;

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseAngle));

    public static Pose2d posEstimate;

    //coordinates for left spike position
    public static double spike1X = -30.4107;
    public static double spike1Y = -7.232;
    public static double spike1Angle = Math.toRadians(90);

    //coordinates for middle spike position
    public static double spike2X = -31.2394;
    public static double spike2Y = -12.1456;
    public static double spike2Angle = 0;

    //coordinates for right spike position
    public static double spike3X = -31.2182;
    public static double spike3Y = -13.9349;
    public static double spike3Angle = Math.toRadians(270);

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

    public static double outtakeHeight = 0;

    // stores the result of Vision locally

    public static RedPipeline.Location positionOfVisionPixel;

    public static double casePos;

    State currentState = State.IDLE;

    OpenCvWebcam camera;
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        RedPipeline vision =  new RedPipeline(telemetry);

        telemetry.addLine("Init Done");
        drive.setPoseEstimate(startPose);
        //still need to enter values for these
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(30)
                .build();
        telemetry.addLine("trajectories built!!!");
        telemetry.addData("cpos", vision.getLocation());
        telemetry.update();

        while(!isStopRequested() && opModeIsActive()){
            posEstimate = drive.getPoseEstimate();

            switch(currentState){
                case IDLE:
                    currentState = State.PARK;

                case PARK:

                    drive.followTrajectorySequence(park3);
                    manip.gateToggle();
                    currentState = State.STOP;
                    break;

                case STOP:
                    posEstimate=drive.getPoseEstimate();
                    break;
            }
        }

    }

}
