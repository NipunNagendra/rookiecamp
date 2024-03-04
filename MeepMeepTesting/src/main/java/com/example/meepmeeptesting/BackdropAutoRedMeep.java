package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;


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

    //coordinates for left spike position
    public static double spike1X = 10.24633638294297;
    public static double spike1Y = -28.8247700133697;
    public static double spike1Angle = Math.toRadians(180);

    //coordinates for middle spike position
    public static double spike2X = 11.612297556497846;
    public static double spike2Y = -32.623006373520104;
    public static double spike2Angle = Math.toRadians(90);

    //coordinates for right spike position
    public static double spike3X = 14.74633638294297;
    public static double spike3Y = -29.9247700133697;
    public static double spike3Angle = Math.toRadians(0);

    public static double backdropMiddleX = 47;
    public static double backdropMiddleY = -35;
    public static double backdropMiddleAngle = Math.toRadians(180);
    public static double backdropLeftStrafe = 8;
    public static double backdropRightStrafe = 8;

    static double backdropLeftX = 47;
    public static double backdropLeftY = -28;
    public static double backdropLeftAngle = Math.toRadians(180);

    public static double backdropRightX = 47;
    public static double backdropRightY = -41;
    public static double backdropRightAngle = Math.toRadians(180);

    public static double preParkY = -58.5;
    public static double outFromBackdrop = 10;
    public static double goingIntoPark = 10;

    public static int outtakeEncoderTicks = 2500;
    public static double temporalMarkerTime = 1.5;

    public static double casenum=1;


    public static String myPosition;

    public static void main(String[] args) {
        MeepMeep meepmeep = new MeepMeep(500);

        RoadRunnerBotEntity myLeftBot = new DefaultBotBuilder(meepmeep)
                // We set this bot to be left
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                .setReversed(true)
//                                .back(5)
                                .splineToSplineHeading(new Pose2d(19.2, -33.3, Math.toRadians(180)), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(spike1X, spike1Y), Math.toRadians(0))
                                .back(5,
                                        SampleMecanumDrive.getVelocityConstraint(10, 2, 14),
                                        SampleMecanumDrive.getAccelerationConstraint(50))
                                .splineToConstantHeading(new Vector2d(backdropMiddleX, backdropMiddleY + backdropLeftStrafe), Math.toRadians(0))

                                //parking in the corner:
                                .setReversed(false)
                                .forward(3)
                                .splineToConstantHeading(new Vector2d(44, -56), Math.toRadians(0))
                                .turn(Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(58.7, -60, Math.toRadians(90)))

                                // alt park
//                                .setReversed(false)
//                                .forward(4.5)
//                                .splineTo(new Vector2d(backdropMiddleX - 10, -20), Math.toRadians(90))
//                                .splineToConstantHeading(new Vector2d(58.7, -11.3), Math.toRadians(0))

                                .build()
                );

        RoadRunnerBotEntity myRightBot = new DefaultBotBuilder(meepmeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                .setReversed(true)
                                .splineTo(new Vector2d(31.5, -31), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(backdropMiddleX, backdropMiddleY - backdropRightStrafe), Math.toRadians(0))

                                //parking in the corner:
//                                .setReversed(false)
//                                .forward(3)
//                                .splineToConstantHeading(new Vector2d(44, -56), Math.toRadians(0))
//                                .turn(Math.toRadians(-90))
//                                .lineToLinearHeading(new Pose2d(58.7, -60, Math.toRadians(90)))

                                // alt park
                                .setReversed(false)
                                .forward(4.5)
                                .splineTo(new Vector2d(backdropMiddleX - 10, -34), Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(backdropMiddleX - 10, -20, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(58.7, -11.3), Math.toRadians(0))

                                .build()
                );

        RoadRunnerBotEntity myMiddleBot = new DefaultBotBuilder(meepmeep)
                // We set this bot to be right
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                .setReversed(true)
//                                .back(12)
                                .splineTo(new Vector2d(12.3, -42.3), Math.toRadians(90))
                                .splineTo(new Vector2d(21.8, -24), Math.toRadians(0))

                                .splineTo(new Vector2d(backdropMiddleX, backdropMiddleY), Math.toRadians(0))

                                //parking in the corner:
                                .setReversed(false)
                                .forward(3)
                                .splineToConstantHeading(new Vector2d(44, -56), Math.toRadians(0))
                                .turn(Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(58.7, -60, Math.toRadians(90)))

                                // alt park
//                                .setReversed(false)
//                                .forward(4.5)
//                                .splineTo(new Vector2d(backdropMiddleX - 10, -20), Math.toRadians(90))
//                                .splineToConstantHeading(new Vector2d(58.7, -11.3), Math.toRadians(0))

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