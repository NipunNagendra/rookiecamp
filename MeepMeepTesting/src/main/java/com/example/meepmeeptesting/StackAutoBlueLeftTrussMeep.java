package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class StackAutoBlueLeftTrussMeep {
    //coordinates for starting position
    public static double startPoseX= -38.15845302224215;
    public static double startPoseY= 65.13672263931143;
    public static double startPoseAngle= Math.toRadians(90);

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;


    //going to left spike position
    public static double moveBackwards1 = 31;
    public static double moveForwards1 = 12;
    public static double turn1 = -90;

    //coordinates for middle spike position
    public static double spike2X =  -37.812297556497846;
    public static double spike2Y = 27.023006373520104;
    public static double spike2Angle = Math.toRadians(270);

    //coordinates for right spike position
    public static double spike3X = -41.64633638294297;
    public static double spike3Y = 32.1247700133697;
    public static double spike3Angle = Math.toRadians(0);

    public static double preTrussX = -38.15845302224215;
    public static double trussX = 20;
    public static double trussY = 57.93672263931143;
    public static double trussAngle = Math.toRadians(180);

    public static double goingDirectlyUnderTruss = 15;
    public static double betweenTruss = 50;
    public static double exitDoor = 15;

    public static double backdropMiddleX = 52;
    public static double backdropMiddleY = 35;
    public static double backdropMiddleAngle = trussAngle;
    public static double backdropLeftStrafe = 6;
    public static double backdropRightStrafe = 6;

    public static double outFromBackdrop = 10;
    public static double preParkY = 58.5;
    public static double goingIntoPark = 15;

    public static int outtakeEncoderTicks = 2500;
    public static double temporalMarkerTime = 1.5;

    public static double casenum=1;

    public static String myPosition;
    public static Boolean danger = Boolean.FALSE;

    public static void main(String[] args) {
        MeepMeep meepmeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepmeep)
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                        .back(moveBackwards1)
                                        .turn(Math.toRadians(turn1))
                                        .forward(moveForwards1)
                                        .back(moveForwards1)
                                        .turn(Math.toRadians(-180))
                                        .lineToLinearHeading(new Pose2d(preTrussX, trussY, trussAngle))
                                        .lineToLinearHeading(new Pose2d(trussX, trussY, trussAngle))
                                        .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                                        .strafeRight(backdropLeftStrafe)
                                        .forward(outFromBackdrop)
                                        .turn(Math.toRadians(90))
                                        .lineToLinearHeading(new Pose2d(backdropMiddleX - outFromBackdrop, preParkY, startPoseAngle - Math.toRadians(180)))
                                        .strafeLeft(goingIntoPark)
                                        .build()


                );


        meepmeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(.95f)
                .addEntity(myBot)
                .start();
    }



}