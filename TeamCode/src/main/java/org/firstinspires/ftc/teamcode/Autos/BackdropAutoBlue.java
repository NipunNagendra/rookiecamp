package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.testing.BluePipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "NearBackdropUnderDoor", group = "Autonomous")
public class BackdropAutoBlue extends LinearOpMode {

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

    Pose2d posEstimate;

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

    public static BluePipeline.Location positionOfVisionPixel;

    State currentState = State.IDLE;
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        BluePipeline vision =  new BluePipeline(telemetry);

        telemetry.addLine("Init Done");
        drive.setPoseEstimate(startPose);
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

        TrajectorySequence park = drive.trajectorySequenceBuilder(posEstimate)
                .lineToC
                .build();

        telemetry.addLine("trajectories built!!!");

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            posEstimate = drive.getPoseEstimate();

            switch(currentState){
                case IDLE:
                    currentState = State.SCORE_PURPLE;

                case SCORE_PURPLE:
                    positionOfVisionPixel = vision.getLocation();
                    if (positionOfVisionPixel == BluePipeline.Location.LEFT) {
                        drive.followTrajectorySequence(scorePurpleLeft);
                    } else if (positionOfVisionPixel == BluePipeline.Location.FRONT) {
                        drive.followTrajectorySequence(scorePurpleMiddle);
                    } else {
                        drive.followTrajectorySequence(scorePurpleRight);
                    }
                    manip.setIntakePower(-0.2);
                    sleep(400);
                    manip.setIntakePower(0);
                    currentState = State.PARK;
                    break;

                case PARK:
                    posEstimate = drive.getPoseEstimate();
                    drive.followTrajectorySequence(park);
                    currentState = State.STOP;
                    break;

                case STOP:
                    break;
            }
        }

    }

}
