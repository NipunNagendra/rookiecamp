package com.example.meepmeeptesting.latest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class BackdropAutoRedSplines2plus2TrussMeep {

    //This is code for BackdropAutoBlue

    //coordinates for start position

    public static double startPoseX = 14.65845302224215;
    public static double startPoseY = -65.13672263931143;
    public static double startPoseAngle = Math.toRadians(270);

    public static Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    //coordinates for left spike position
    public static double spike1ForwardAmount = 5;
    public static double preSpike1X = 19.2;
    public static double preSpike1Y = -37.8;
    public static double preSpike1Angle = Math.toRadians(180);
    public static double spike1X = 13.25 - spike1ForwardAmount;
    public static double spike1Y = -33.8;

    //coordinates for middle spike position
    public static double preSpike2X = 12.3;
    public static double preSpike2Y = -42.3;
    public static double spike2X = 21.8;
    public static double spike2Y = -24;

    //coordinates for right spike position
    public static double spike3X = 31.5;
    public static double spike3Y = -31;
    public static double spike3BackAmount = 3;

    //coordinates for backdrop positions
    public static double backdropMiddleX = 47;
    public static double backdropMiddleY = -35;
    public static double backdropMiddleAngle = Math.toRadians(180);
    public static double backdropBackAmount = 5;
    public static double cycleBackdropBackAmount = 4;

    public static double backdropLeftStrafe = 4;
    public static double backdropRightStrafe = 4;

    //coordinates for cycling
    public static double preTrussX = 20;
    public static double preTrussY = -58.5;
    public static double underTrussX = -47;
    public static double underTrussY = preTrussY;
    public static double postTrussX = -57;
    public static double postTrussY = -46.5;
    public static double trussStackX = -60;
    public static double trussStackY = -35.5;
    public static double trussStackForwardAmount = 2;

    //intake and outtake movement
    public static int outtakeEncoderTicksUp = 2500;
    public static double temporalMarkerTimeUp = 1.5;
    public static int outtakeEncoderTicksDown = 0;
    public static double temporalMarkerTimeDown = 1;

    //coordiantes for park in corner
    public static double parkForwardAmount = 3;
    public static double parkStrafeAmount = 25;

    //coordiantes for alt park
    public static double altParkForwardAmount = 4.5;
    public static double preAltParkY = -20;
    public static double preAltParkXChange = -10;
    public static double altParkX = 58.7, altParkY = -12.3;

    public static double casenum = 1;

    public static int myPosition = 1;
    public static double underDoorWaitTime = 0.01;
    public static double altParkWaitTime = 0.01;


    public static double temporalMarkerTimeDOWN = .5;
    public static double temporalMarkerTimeUP = 1;


    public static int outtakeEncoderTicks = 2000;
    public static int outtakeOG = 0;

    public static void main(String[] args) {
        MeepMeep meepmeep = new MeepMeep(500);

        RoadRunnerBotEntity myLeftBot = new DefaultBotBuilder(meepmeep)
                // We set this bot to be left
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .setReversed(true)
                                .back(1)
                                .splineToSplineHeading(new Pose2d(preSpike1X, preSpike1Y, preSpike1Angle), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(spike1X, spike1Y), Math.toRadians(0))
                                .back(spike1ForwardAmount)

                                .splineToConstantHeading(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY + backdropLeftStrafe), Math.toRadians(0))
                                .back(backdropBackAmount)

                                //cycling
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(preTrussX, preTrussY), Math.toRadians(180))
                                .lineToConstantHeading(new Vector2d(underTrussX, underTrussY))
                                .splineToConstantHeading(new Vector2d(postTrussX, postTrussY), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(trussStackX, trussStackY), Math.toRadians(180))
                                .forward(trussStackForwardAmount)
                                .setReversed(true)
                                .back(trussStackForwardAmount)
                                .splineToConstantHeading(new Vector2d(postTrussX, postTrussY), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(underTrussX, underTrussY), Math.toRadians(0))
                                .lineToConstantHeading(new Vector2d(preTrussX, preTrussY))
                                .splineToConstantHeading(new Vector2d(backdropMiddleX - cycleBackdropBackAmount, backdropMiddleY - backdropRightStrafe), Math.toRadians(0))
                                .back(cycleBackdropBackAmount)

                                //parking in the corner:
                                .setReversed(false)
                                .forward(parkForwardAmount)
                                .strafeLeft(parkStrafeAmount)

                                .build()
                );

        RoadRunnerBotEntity myMiddleBot = new DefaultBotBuilder(meepmeep)
                // We set this bot to be right
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startPose)
                                        .setReversed(true)
                                        .splineTo(new Vector2d(preSpike2X, preSpike2Y), Math.toRadians(90))
                                        .splineTo(new Vector2d(spike2X, spike2Y), Math.toRadians(0))

                                        .splineTo(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY), Math.toRadians(0))
                                        .back(backdropBackAmount)

                                        //cycling
                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(preTrussX, preTrussY), Math.toRadians(180))
                                        .lineToConstantHeading(new Vector2d(underTrussX, underTrussY))
                                        .splineToConstantHeading(new Vector2d(postTrussX, postTrussY), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(trussStackX, trussStackY), Math.toRadians(180))
                                        .forward(trussStackForwardAmount)
                                        .setReversed(true)
                                        .back(trussStackForwardAmount)
                                        .splineToConstantHeading(new Vector2d(postTrussX, postTrussY), Math.toRadians(270))
                                        .splineToConstantHeading(new Vector2d(underTrussX, underTrussY), Math.toRadians(0))
                                        .lineToConstantHeading(new Vector2d(preTrussX, preTrussY))
                                        .splineToConstantHeading(new Vector2d(backdropMiddleX - cycleBackdropBackAmount, backdropMiddleY - backdropRightStrafe), Math.toRadians(0))
                                        .back(cycleBackdropBackAmount)

                                        //parking in the corner:
                                        .setReversed(false)
                                        .forward(parkForwardAmount)
                                        .strafeLeft(parkStrafeAmount)

                                        .build()
                );

        RoadRunnerBotEntity myRightBot = new DefaultBotBuilder(meepmeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startPose)
                                        .setReversed(true)
                                        .splineTo(new Vector2d(spike3X, spike3Y), Math.toRadians(0))
                                        .back(spike3BackAmount)

                                        .splineToConstantHeading(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY - backdropRightStrafe), Math.toRadians(0))
                                        .back(backdropBackAmount)

                                        //cycling
                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(preTrussX, preTrussY), Math.toRadians(180))
                                        .lineToConstantHeading(new Vector2d(underTrussX, underTrussY))
                                        .splineToConstantHeading(new Vector2d(postTrussX, postTrussY), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(trussStackX, trussStackY), Math.toRadians(180))
                                        .forward(trussStackForwardAmount)
                                        .setReversed(true)
                                        .back(trussStackForwardAmount)
                                        .splineToConstantHeading(new Vector2d(postTrussX, postTrussY), Math.toRadians(270))
                                        .splineToConstantHeading(new Vector2d(underTrussX, underTrussY), Math.toRadians(0))
                                        .lineToConstantHeading(new Vector2d(preTrussX, preTrussY))
                                        .splineToConstantHeading(new Vector2d(backdropMiddleX - cycleBackdropBackAmount, backdropMiddleY - backdropRightStrafe), Math.toRadians(0))
                                        .back(cycleBackdropBackAmount)

                                        //parking in the corner:
                                        .setReversed(false)
                                        .forward(parkForwardAmount)
                                        .strafeLeft(parkStrafeAmount)

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