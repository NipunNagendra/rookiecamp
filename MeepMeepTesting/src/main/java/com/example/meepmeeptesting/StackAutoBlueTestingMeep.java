package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class StackAutoBlueTestingMeep {

    //coordinates for starting position (0, 0, 0)
    public static double startPoseX= -38.15845302224215;
    public static double startPoseY= 65.13672263931143;
    public static double startPoseAngle= Math.toRadians(90);

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    //coordinates for right spike position
    public static double spike3X = -42.64633638294297;
    public static double spike3Y = 31.1247700133697;
    public static double spike3Angle = Math.toRadians(180);

    //coordinates for middle spike position
    public static double spike2X =  -37.812297556497846;
    public static double spike2Y = 27.023006373520104;
    public static double spike2Angle = Math.toRadians(270);

    //coordinates for left spike position
//    public static double spike3X = -34.26642694740993;
//    public static double spike3Y = 29.54644728121096;
//    public static double spike3Angle = Math.toRadians(180);
    public static double moveBackwards1 = 31;
    public static double moveForward1 = 12;
    public static double turn1 = -90;

    public static double preTrussX = -38.15845302224215;
    public static double trussX = 20;
    public static double dangerPathX = -15;
    public static double trussY = 56.33672263931143;
    public static double trussAngle = Math.toRadians(180);

    public static double goingDirectlyUnderTruss = 15;
    public static double betweenTruss = 35;
    public static double exitDoor = 15;

    public static double backdropMiddleX = 54.5;
    public static double backdropMiddleY = 33;
    public static double backdropMiddleAngle = trussAngle;
    public static double backdropLeftStrafe = 5;
    public static double backdropRightStrafe = 5;
    public static double preParkY = 55;
    public static double goingIntoPark = 18;
    public static double temporalMarkerTime = 1.5;

    public static double temporalMarkerTimeAlternate = 4;

    public static double outFromBackdrop = 10;

    public static int outtakeEncoderTicksUp = 1800;
    public static int outtakeEncoderTicksDown = 0;
    public static double temporalMarkerTimeUp = 1.5;
    public static double temporalMarkerTimeDown = 0.5;

    public static void main(String[] args) {
        MeepMeep meepmeep = new MeepMeep(800);

        RoadRunnerBotEntity myMiddleBot = new DefaultBotBuilder(meepmeep)
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                .lineToLinearHeading(new Pose2d(spike2X, spike2Y, spike2Angle))
                                .back(5)
                                .lineToLinearHeading(new Pose2d(preTrussX, trussY, trussAngle))
                                .lineToLinearHeading(new Pose2d(trussX, trussY, trussAngle))
                                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                                .forward(outFromBackdrop)
                                .lineToLinearHeading(new Pose2d(backdropMiddleX - outFromBackdrop, preParkY, startPoseAngle + Math.toRadians(90)))
                                .turn(Math.toRadians(90))
                                .strafeLeft(goingIntoPark)
                                .build()


                );

        RoadRunnerBotEntity myRightBot = new DefaultBotBuilder(meepmeep)
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                .lineToLinearHeading(new Pose2d(spike3X, spike3Y, spike3Angle))
                                .back(startPoseX - spike3X)
                                .lineToLinearHeading(new Pose2d(preTrussX, trussY, trussAngle))
                                .lineToLinearHeading(new Pose2d(trussX, trussY, trussAngle))
                                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                                .strafeLeft(backdropRightStrafe)
                                .forward(outFromBackdrop)
                                .turn(Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(backdropMiddleX - outFromBackdrop, preParkY, startPoseAngle - Math.toRadians(180)))
                                .strafeLeft(goingIntoPark)
                                .build()


                );

        RoadRunnerBotEntity myLeftBot = new DefaultBotBuilder(meepmeep)
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                .back(moveBackwards1)
                                .turn(Math.toRadians(turn1))
                                .forward(moveForward1)
                                .back(10)
                                .turn(Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(preTrussX, trussY, trussAngle))
                                .lineToLinearHeading(new Pose2d(trussX, trussY, trussAngle))
                                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                                .strafeRight(backdropLeftStrafe)
                                .forward(outFromBackdrop)
                                .lineToLinearHeading(new Pose2d(backdropMiddleX - outFromBackdrop, preParkY, startPoseAngle + Math.toRadians(90)))
                                .turn(Math.toRadians(90))
                                .strafeLeft(goingIntoPark)
                                .build()


                );


        meepmeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(.95f)
                .addEntity(myMiddleBot)
                .addEntity(myRightBot)
                .addEntity(myLeftBot)
                .start();
    }
}
