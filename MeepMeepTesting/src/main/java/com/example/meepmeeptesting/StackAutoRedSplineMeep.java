package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

// TODO:
//  CHANGE VAR NAMES FOR SPLINE TO BACKDROP, ADD PARK,
//  MAKE SPLINE TO BACKDROP NOT GO INTO BLUE HUMAN PLAYER STATION,
//  MAKE SPLINE TO BACKDROP NOT TOUCH SPIKE MARK AS MUCH


public class StackAutoRedSplineMeep {
    MeepMeep meepmeep = new MeepMeep(600);

    //coordinates for starting position (0, 0, 0)
    public static double startPoseX= -37.95845302224215;
    public static double startPoseY= 65.63672263931143;
    public static double startPoseAngle= Math.toRadians(90);

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    //coordinates for left spike position
    public static double spike1X = -42.64633638294297;
    public static double spike1Y = 31.1247700133697;
    public static double spike1Angle = Math.toRadians(0);
    public static double spike1Strafe = 10;
    public static double spike1BackAmount = 3;
    public static double spike1StackOffset = 5;

    //coordinates for middle spike position
    public static double spike2X =  -37.812297556497846;
    public static double spike2Y = 27.023006373520104;
    public static double spike2Angle = Math.toRadians(270);

    //coordinates for right spike position
    public static double spike3X = -32.76642694740993;
    public static double spike3Y = 34.04644728121096;
    public static double spike3Angle = Math.toRadians(180);

//    public static double moveBackwards3 = 31;
//    public static double moveForward3 = 11;
//    public static double turn3 = 90;

    public static double firstStackX = -60;
    public static double firstStackY = 35;
    public static double firstStackAngle = Math.toRadians(0);

    public static double prePreTrussX = -55.7;
    public static double prePreTrussY = 39;
    public static double preTrussX = -32.5;
    public static double trussX = 27;
    public static double trussY = 58.5;
    public static double trussAngle = Math.toRadians(0);

    public static double preBackdropX = 42;
    public static double preBackdropY = 35.30;

    public static double backdropMiddleX = 46;
    public static double backdropMiddleY = 35.3;
    public static double backdropMiddleAngle = trussAngle;
    public static double backdropLeftStrafe = 8;
    public static double backdropRightStrafe = 8;

    public static double preParkX = 46.5;
    public static double preParkY = 58;

    public static double moveBackwards3 = 31;
    public static double moveForward3 = 12;
    public static double turn3 = 90;
    public static double dangerPathX = -15;

    public static double goingDirectlyUnderTruss = 15;
    public static double betweenTruss = 35;
    public static double exitDoor = 15;

    public static double goingIntoPark = 18;
    public static double temporalMarkerTime = 1.5;

    public static double temporalMarkerTimeAlternate = 4;

    public static double outFromBackdrop = 10;

    public static int outtakeEncoderTicksUp = 1800;
    public static int outtakeEncoderTicksDown = 0;
    public static double temporalMarkerTimeUp = 1.5;
    public static double temporalMarkerTimeDown = 0.5;

    public static double casenum=1;


    public static String myPosition;

    public static void main(String[] args) {
        MeepMeep meepmeep = new MeepMeep(600);

//        RoadRunnerBotEntity myContainingEverythingBot = new DefaultBotBuilder(meepmeep)
//                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
//                .followTrajectorySequence(drive ->

                        // to right spike to stack
//                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(spike3X, spike3Y, spike3Angle), Math.toRadians(0))
//                                .splineToSplineHeading(new Pose2d(firstStackX + 10, firstStackY, firstStackAngle), Math.toRadians(180))
//                                .forward(10)
//                                .setReversed(false)
//                                .build()

                        // to middle spike to stack
//                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
//                                .setReversed(true)
//                                .lineToLinearHeading(new Pose2d(spike2X, spike2Y, spike2Angle))
//                                .back(5)
//                                .splineToSplineHeading(new Pose2d(firstStackX + 10, firstStackY, firstStackAngle), Math.toRadians(180))
//                                .forward(10)
//                                .setReversed(false)
//                                .build()

                        // to left spike to stack
//                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(spike1X, spike1Y - 10, spike1Angle), Math.toRadians(90))
//                                .strafeRight(10)
//                                .back(3)
//                                .splineToConstantHeading(new Vector2d(spike1X + 3, firstStackY - 5), Math.toRadians(180))
//                                .forward(16)
//                                .splineToConstantHeading(new Vector2d(firstStackX + 1, firstStackY), Math.toRadians(180))
//                                .forward(1)
//                                .setReversed(false)
//                                .build()

                        // stack to backdrop spline sequence
//                        drive.trajectorySequenceBuilder(new Pose2d(-67.12, -36.14, Math.toRadians(180)))
//                                .setReversed(true)
//                                .splineToConstantHeading(new Vector2d(-36.98, -58.5), Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(23, -58.5), Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(47.22, -35.30), Math.toRadians(0))
//                                .build()



//                );

        RoadRunnerBotEntity myLeftBot = new DefaultBotBuilder(meepmeep)
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->

                                // left
                                drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                        // to left spike
                                        .lineToLinearHeading(new Pose2d(spike1X, spike1Y, spike1Angle))
//                                        .strafeRight(10)


                                        // to backdrop
                                        .setReversed(true)
                                        .back(5)
                                        .splineTo(new Vector2d(preTrussX, trussY), Math.toRadians(0))
//                .lineToLinearHeading(new Pose2d(preTrussX + 1, trussY, trussAngle))
                                        .lineTo(new Vector2d(trussX, trussY))
                                        .splineTo(new Vector2d(preBackdropX, preBackdropY), Math.toRadians(0))

                                        .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY + backdropLeftStrafe, backdropMiddleAngle))

                                        .forward(3)
                                        .splineToConstantHeading(new Vector2d(preParkX, preParkY), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(preParkX + 1, preParkY, Math.toRadians(90)), Math.toRadians(0))
                                        .turn(Math.toRadians(-90))
                                        .strafeRight(10)

                                        .build()
                );

        RoadRunnerBotEntity myMiddleBot = new DefaultBotBuilder(meepmeep)
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->

                                drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                        .setReversed(true)
                                        .lineToLinearHeading(new Pose2d(spike2X, spike2Y, spike2Angle))
//                                        .strafeRight(10)

                                        .back(5)
                                        .splineTo(new Vector2d(preTrussX, trussY), Math.toRadians(0))
                                        .lineTo(new Vector2d(trussX, trussY))
                                        .splineTo(new Vector2d(preBackdropX, preBackdropY), Math.toRadians(0))

                                        .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                                        .forward(3)
                                        .splineToConstantHeading(new Vector2d(preParkX, preParkY), Math.toRadians(0))

                                        .turn(Math.toRadians(-90))
                                        .strafeRight(10)
                                        .build()

                );


        RoadRunnerBotEntity myRightBot = new DefaultBotBuilder(meepmeep)
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->

                                drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))

                                        .back(moveBackwards3)
                                        .turn(Math.toRadians(turn3))
                                        .forward(moveForward3)

                                        .setReversed(true)
                                        .back(10)
                                        .splineTo(new Vector2d(preTrussX, trussY), Math.toRadians(0))
                                        .lineTo(new Vector2d(trussX, trussY))
                                        .splineTo(new Vector2d(preBackdropX, preBackdropY), Math.toRadians(0))

                                        .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY - backdropRightStrafe, backdropMiddleAngle))

                                        .forward(3)
                                        .splineToConstantHeading(new Vector2d(preParkX, preParkY), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(preParkX + 1, preParkY, Math.toRadians(90)), Math.toRadians(0))
                                        .turn(Math.toRadians(-90))
                                        .strafeRight(10)

                                        .build()

                );


        meepmeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(.95f)
                .addEntity(myLeftBot)
                .addEntity(myMiddleBot)
                .addEntity(myRightBot)
                .start();
    }



}