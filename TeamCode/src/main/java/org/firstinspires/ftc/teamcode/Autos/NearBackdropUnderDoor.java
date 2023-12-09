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
public class NearBackdropUnderDoor extends LinearOpMode {

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
    double startPoseX= 0;
    double startPoseY= 0;
    double startPoseAngle= 0;

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseAngle));

    Pose2d posEstimate;

    //coordinates for left spike position
    int spike1X = 0;
    int spike1Y = 0;
    int spike1Angle = 0;

    //coordinates for middle spike position
    int spike2X = 0;
    int spike2Y = 0;
    int spike2Angle = 0;

    //coordinates for right spike position
    int spike3X = 0;
    int spike3Y = 0;
    int spike3Angle = 0;

    //coordinates for left backdrop position
    int backDrop1X = 0;
    int backDrop1Y = 0;
    int backDrop1Angle = 0;

    //coordinates for middle backdrop position
    int backDrop2X = 0;
    int backDrop2Y = 0;
    int backDrop2Angle = 0;

    // double variable for moving back right before going to the right backdrop
    double avoidPixelBack = 0;

    //coordinates for right backdrop position
    int backDrop3X = 0;
    int backDrop3Y = 0;
    int backDrop3Angle = 0;

    //coordinates for door position
    int doorX = 0;
    int doorY = 0;
    int doorAngle = 0;

    //coordinates for white stack position
    int whiteStackX = 0;
    int whiteStackY = 0;
    int whiteStackAngle = 0;

    //coordinates for position right before going closer to the wall for full park
    int subParkX = 0;
    int subParkY = 0;
    int subParkAngle = 0;

    //coordinates for park position
    int parkX = 0;
    int parkY = 0;
    int parkAngle = 0;

    int outtakeHeight = 0;

    // stores the result of Vision locally

    public static BluePipeline.Location positionOfVisionPixel;

    State currentState = State.IDLE;
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        BluePipeline vision =  new BluePipeline(telemetry);

        telemetry.addLine("Init Done");

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

        //still need to enter values for these
        TrajectorySequence purplePixelToBackDropLeft  = drive.trajectorySequenceBuilder(new Pose2d(spike1X, spike2Y, spike1Angle))
                .lineToLinearHeading(new Pose2d(backDrop1X, backDrop1Y, backDrop1Angle))
                .build();

        //still need to enter values for these
        TrajectorySequence purplePixelToBackDropMiddle  = drive.trajectorySequenceBuilder(new Pose2d(spike2X, spike2Y, spike2Angle))
                .lineToLinearHeading(new Pose2d(backDrop2X , backDrop2Y, backDrop2Angle))
                .build();

        //still need to enter values for these
        TrajectorySequence purplePixelToBackDropRight  = drive.trajectorySequenceBuilder(new Pose2d(spike3X, spike3Y, spike3Angle))
                .back(avoidPixelBack)
                .lineToLinearHeading(new Pose2d(backDrop3X, backDrop3Y, backDrop3Angle))
                .build();

//        //still need to enter values for these
//        TrajectorySequence backDropToStack =  drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(doorX, doorY, doorAngle))
//                .lineToLinearHeading(new Pose2d(whiteStackX, whiteStackY, whiteStackAngle))
//                .build();

//        //still need to enter values for these
//        TrajectorySequence stackToBackDropLeft =  drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(doorX, doorY, doorAngle))
//                .lineToLinearHeading(new Pose2d(backDrop1X, backDrop1Y, backDrop1Angle))
//                .build();
//
//        //still need to enter values for these
//        TrajectorySequence stackToBackDropMiddle =  drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(doorX, doorY, doorAngle))
//                .lineToLinearHeading(new Pose2d(backDrop2X, backDrop2Y, backDrop2Angle))
//                .build();
//
//        //still need to enter values for these
//        TrajectorySequence stackToBackDropRight =  drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(doorX, doorY, doorAngle))
//                .lineToLinearHeading(new Pose2d(backDrop3X, backDrop3Y, backDrop3Angle))
//                .build();

        //still need to enter values for these
        TrajectorySequence backDropToPark = drive.trajectorySequenceBuilder(posEstimate)
                .lineToLinearHeading(new Pose2d(subParkX, subParkY, subParkAngle))
                .lineToLinearHeading(new Pose2d(parkX, parkY, parkAngle))
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
                    currentState = State.SCORE_YELLOW;
                    break;

                case SCORE_YELLOW:
                    if (positionOfVisionPixel == BluePipeline.Location.LEFT) {
                        drive.followTrajectorySequence(purplePixelToBackDropLeft);
                    } else if (positionOfVisionPixel == BluePipeline.Location.FRONT) {
                        drive.followTrajectorySequence(purplePixelToBackDropMiddle);
                    } else {
                        drive.followTrajectorySequence(purplePixelToBackDropRight);
                    }
                    manip.moveOuttakeLift(outtakeHeight);
                    //manip.gateToggle(false);
                    sleep(100);
                    //manip.gateToggle(true);
                    manip.bottomOutLift();
                    currentState = State.PARK;
                    break;

//                case INTAKE_WHITE:
//                    drive.followTrajectorySequence(backDropToStack);
//                    manip.setIntakePower(.5);
//                    sleep(2000);
//                    manip.setIntakePower(0);
//                    currentState = State.PARK;
//                    break;

                case PARK:
                    posEstimate = drive.getPoseEstimate();
                    drive.followTrajectorySequence(backDropToPark);
                    currentState = State.STOP;
                    break;

                case STOP:
                    break;
            }
        }

    }

}
