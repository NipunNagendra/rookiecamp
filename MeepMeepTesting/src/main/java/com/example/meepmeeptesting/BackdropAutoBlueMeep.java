package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class BackdropAutoBlueMeep {
    //This is code for BackdropAutoBlue
    //coordinates for starting position (0, 0, 0)
    public static double startPoseX = 14.65845302224215;
    public static double startPoseY = 65.13672263931143;
    public static double startPoseAngle = Math.toRadians(90);

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    // TODO: TUNE ALL POSITIONS

    //coordinates for left spike position

    public static double moveBackwards3 = 31;
    public static double moveBackwardsTowardBackDrop = 34;
    public static double turn3 = 90;

    //coordinates for right spike position
    public static double spike3X = 11.35845302224215;
    public static double spike3Y = 30.44644728121096;
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

    public static double backdropLeftX = 44;
    public static double backdropLeftY = 44;
    public static double backdropLeftAngle = Math.toRadians(180);

    public static double backdropRightX = 45;
    public static double backdropRightY = 29;
    public static double backdropRightAngle = Math.toRadians(180);
    public static double strafeForPark = 20;
    public static double backdropRightStrafe = 8;

    public static double casenum=1;


    public static String myPosition;

    public static double strafeToTag = 8;

    public static double temporalMarkerTimeDOWN = 1.5;
    public static double temporalMarkerTimeUP = 5;


    public static int outtakeEncoderTicks = 2000;
    public static int outtakeOG = 0;

    // stores the result of Vision locally


    public static double casePos;

    public static void main(String[] args) {
        MeepMeep meepmeep = new MeepMeep(600);

        RoadRunnerBotEntity myRightBot = new DefaultBotBuilder(meepmeep)
                // We set this bot to be left
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                .lineToLinearHeading(new Pose2d(spike1X, spike1Y, spike1Angle))
                                .forward(3)
                                .back(5)
                                .strafeLeft(20)
                                .lineToLinearHeading(new Pose2d(backdropLeftX, backdropLeftY, backdropLeftAngle))
                                .forward(5)
                                .strafeRight(15)
                                .turn(Math.toRadians(90))
                                .strafeLeft(15)
                                .build()
                );

        RoadRunnerBotEntity myMiddleBot = new DefaultBotBuilder(meepmeep)
                // We set this bot to be middle
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                .back(7)
                                //.turn(Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(spike2X, spike2Y, spike2Angle))
                                .forward(3)
                                .back(7)
                                .turn(Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                                .forward(5)
                                //.strafeRight(8)
                                .strafeRight(24)
                                .turn(Math.toRadians(90))
                                .strafeLeft(15)
                                .build()
                );

        RoadRunnerBotEntity myLeftBot = new DefaultBotBuilder(meepmeep)
                // We set this bot to be right
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                .back(10)
                               // .turn(Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(spike3X, spike3Y, spike3Angle))
                                .forward(3)

                                .lineToLinearHeading(new Pose2d(backdropRightX, backdropRightY, backdropRightAngle))
                                .forward(5)
                                .strafeRight(30)
                                .turn(Math.toRadians(90))
                                .strafeLeft(15)
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