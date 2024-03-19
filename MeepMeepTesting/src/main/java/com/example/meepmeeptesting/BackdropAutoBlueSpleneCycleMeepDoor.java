package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

//THIS IS THE GOOD ONE, BACKDROPAUTOBLUESPLINEPUSHMEEP IS TRASH
public class BackdropAutoBlueSpleneCycleMeepDoor {
    //This is code for BackdropAutoBlue
    //coordinates for starting position (0, 0, 0)


    //coordinates for starting position (0, 0, 0)
    public static double startPoseX = 14.65845302224215;
    public static double startPoseY = 65.13672263931143;
    public static double startPoseAngle = Math.toRadians(90);

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    // TODO: TUNE ALL POSITIONS

    //going to left spike position
    public static double moveBackwards1 = 31;
    public static double moveForwards1 = 12;
    public static double turn1 = -90;

    //coordinates for left spike position
    public static double spike1X = 6.74633638294297;
    public static double spike1Y = 28.8247700133697;
    public static double spike1Angle = Math.toRadians(180);

    //coordinates for middle spike position
    public static double spike2X = 20;
    public static double spike2Y = 25.08633;
    public static double spike2Angle = Math.toRadians(180);

    //coordinates for right spike position
    public static double spike3X = 28.74633638294297;
    public static double spike3Y = 29.9247700133697;
    public static double spike3Angle = Math.toRadians(180);

    public static double backdropMiddleX = 47;
    public static double backdropMiddleY = 35;
    public static double backdropMiddleAngle = Math.toRadians(180);
    public static double backdropLeftStrafe = 4;
    public static double backdropRightStrafe = 4;

    static double backdropLeftX = 47;
    public static double backdropLeftY = 41;
    public static double backdropLeftAngle = Math.toRadians(180);

    public static double backdropRightX = 47;
    public static double backdropRightY = 28;
    public static double backdropRightAngle = Math.toRadians(180);


    public static double preParkY = -53;
    public static double goingIntoPark = 10;

    public static double temporalMarkerTimeDOWN = .5;
    public static double temporalMarkerTimeUP = 1;


    public static int outtakeEncoderTicks = 2000;
    public static int outtakeOG = 0;

    public static double casenum=1;


    public static String myPosition;

    public static double casePos;



    public static double color = 1;


    public static void main(String[] args) {
        MeepMeep meepmeep = new MeepMeep(500);

        RoadRunnerBotEntity myLeftBot = new DefaultBotBuilder(meepmeep)
                // We set this bot to be left
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(spike1X+12, spike1Y), Math.toRadians(0))
                                        .forward(9)
                                        .back(8)
                                        .splineTo(new Vector2d(backdropLeftX, backdropLeftY), Math.toRadians(0))

                                        .setReversed(false)
                                        .splineTo(new Vector2d(20, 12), Math.toRadians(180))
                                        .splineTo(new Vector2d(-50, 12), Math.toRadians(180))
                                        .forward(10)
//
                                        .setReversed(true)
                                        .back(10)
                                        .splineTo(new Vector2d(20, 12), Math.toRadians(0))
                                        .splineTo(new Vector2d(backdropLeftX, backdropLeftY), Math.toRadians(0))

                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(38, 11.3), Math.toRadians(270))
                                        .lineToLinearHeading(new Pose2d(60, 11.3, Math.toRadians(270)))
                                        .build()
                );

        RoadRunnerBotEntity myMiddleBot = new DefaultBotBuilder(meepmeep)
                // We set this bot to be middle
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(50, 50, 2.4242626190185548, Math.toRadians(214.79), 16)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(spike2X+5, spike2Y), Math.toRadians(0))
                                        .splineTo(new Vector2d(backdropMiddleX, backdropMiddleY), Math.toRadians(0))

                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(38, 16), Math.toRadians(180))
                                        .splineTo(new Vector2d(-50, 12), Math.toRadians(180))
                                        .forward(10)
//
                                        .setReversed(true)
                                        .back(10)
                                        .splineTo(new Vector2d(38, 16), Math.toRadians(0))
                                        .splineToConstantHeading(new Vector2d(backdropMiddleX, backdropMiddleY), Math.toRadians(0))

                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(38, 11.3), Math.toRadians(270))
                                        .lineToLinearHeading(new Pose2d(60, 11.3, Math.toRadians(270)))
                                        .build()
                );

        RoadRunnerBotEntity myRightBot = new DefaultBotBuilder(meepmeep)
                // We set this bot to be right
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(50, 50, 2.4242626190185548, Math.toRadians(214.79), 16)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(spike3X, spike3Y), Math.toRadians(0))
                                        .splineTo(new Vector2d(backdropRightX, backdropRightY), Math.toRadians(0))

                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(38, 16), Math.toRadians(180))
                                        .splineTo(new Vector2d(-50, 12), Math.toRadians(180))
                                        .forward(10)
//
                                        .setReversed(true)
                                        .back(10)
                                        .splineTo(new Vector2d(38, 16), Math.toRadians(0))
                                        .splineToConstantHeading(new Vector2d(backdropRightX, backdropRightY), Math.toRadians(0))

                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(38, 11.3), Math.toRadians(270))
                                        .lineToLinearHeading(new Pose2d(60, 11.3, Math.toRadians(270)))
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