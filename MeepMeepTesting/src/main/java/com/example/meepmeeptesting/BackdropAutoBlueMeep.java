package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class BackdropAutoBlueMeep {
    //This is code for BackdropAutoBlue
    //coordinates for starting position (0, 0, 0)
    public static double startPoseX= 11.35845302224215;
    public static double startPoseY= 65.13672263931143;
    public static double startPoseAngle= Math.toRadians(90);

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    public static double moveBackwards3 = 31;
    public static double moveBackwardsTowardBackDrop = 34;
    public static double turn3 = 90;

    //coordinates for right spike position
    public static double spike3X = 11.35845302224215;
    public static double spike3Y = 34.44644728121096;
    public static double spike3Angle = Math.toRadians(180);

    //coordinates for left spike position
    public static double spike1X = 11.35845302224215;
    public static double spike1Y = 34.44644728121096;
    public static double spike1Angle = Math.toRadians(0);

    //coordinates for middle spike position
    public static double spike2X =  11.35845302224215;
    public static double spike2Y = 34.44644728121096;
    public static double spike2Angle = Math.toRadians(270);

    public static double preTrussX = -38.15845302224215;
    public static double trussX = 15;
    public static double trussY = -55.93672263931143;
    public static double trussAngle = Math.toRadians(180);
    public static double backdropMiddleX = 45;
    public static double backdropMiddleY = 34;
    public static double backdropMiddleAngle = Math.toRadians(180);
    public static double strafeForPark = 20;
    public static double backdropRightStrafe = 8;

    public static double casenum=1;


    public static String myPosition;

    public static void main(String[] args) {
        MeepMeep meepmeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepmeep)
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                .back(7)
                                .turn(Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(spike2X, spike2Y, spike2Angle))
                                .back(7)
                                .turn(Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                                .forward(5)
                                .strafeRight(8)
                                .strafeRight(16)
                                .turn(Math.toRadians(90))
                                .strafeLeft(15)
                                .build()


                );


        meepmeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(.95f)
                .addEntity(myBot)
                .start();
    }
}