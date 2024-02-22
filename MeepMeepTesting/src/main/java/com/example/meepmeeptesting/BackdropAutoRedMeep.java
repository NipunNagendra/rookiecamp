package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class BackdropAutoRedMeep {
    //This is code for BackdropAutoBlue
    //coordinates for starting position (0, 0, 0)
    public static double startPoseX = 14.65845302224215;
    public static double startPoseY = -65.13672263931143;
    public static double startPoseAngle = Math.toRadians(270);

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    // TODO: TUNE ALL POSITIONS

    //going to left spike position
    public static double moveBackwards1 = 31;
    public static double moveForwards1 = 12;
    public static double turn1 = -90;

    //coordinates for middle spike position
    public static double spike2X = -37.812297556497846;
    public static double spike2Y = 27.023006373520104;
    public static double spike2Angle = Math.toRadians(270);

    //coordinates for right spike position
    public static double spike3X = -41.64633638294297;
    public static double spike3Y = 32.1247700133697;
    public static double spike3Angle = Math.toRadians(0);

    public static double backdropMiddleX = 52;
    public static double backdropMiddleY = -35;
    public static double backdropMiddleAngle = Math.toRadians(180);
    public static double backdropLeftStrafe = 4;
    public static double backdropRightStrafe = 4;

    public static double preParkY = -58.5;
    public static double outFromBackdrop = 10;
    public static double goingIntoPark = 10;

    public static int outtakeEncoderTicks = 2500;
    public static double temporalMarkerTime = 1.5;

    public static double casenum=1;


    public static String myPosition;

    public static void main(String[] args) {
        MeepMeep meepmeep = new MeepMeep(800);

        RoadRunnerBotEntity myLeftBot = new DefaultBotBuilder(meepmeep)
                // We set this bot to be left
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                .back(moveBackwards1)
                                .turn(Math.toRadians(turn1))
                                .forward(moveForwards1)
                                .back(moveForwards1)
                                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                                .strafeRight(backdropLeftStrafe)
                                .forward(outFromBackdrop)
                                .turn(Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(backdropMiddleX - outFromBackdrop, preParkY, startPoseAngle - Math.toRadians(180)))
                                .strafeRight(goingIntoPark)
                                .build()
                );

        RoadRunnerBotEntity myMiddleBot = new DefaultBotBuilder(meepmeep)
                // We set this bot to be middle
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                .lineToLinearHeading(new Pose2d(spike2X, spike2Y, spike2Angle))
                                .back(5)
                                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                                .forward(outFromBackdrop)
                                .turn(Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(backdropMiddleX - outFromBackdrop, preParkY, startPoseAngle - Math.toRadians(180)))
                                .strafeRight(goingIntoPark)
                                .build()
                );

        RoadRunnerBotEntity myRightBot = new DefaultBotBuilder(meepmeep)
                // We set this bot to be right
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                .lineToLinearHeading(new Pose2d(spike3X, spike3Y, spike3Angle))
                                .back(startPoseX - spike3X)
                                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                                .strafeLeft(backdropRightStrafe)
                                .forward(outFromBackdrop)
                                .turn(Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(backdropMiddleX - outFromBackdrop, preParkY, startPoseAngle - Math.toRadians(180)))
                                .strafeRight(goingIntoPark)
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