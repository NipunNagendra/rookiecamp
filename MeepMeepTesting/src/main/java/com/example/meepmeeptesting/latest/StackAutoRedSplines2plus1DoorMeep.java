package com.example.meepmeeptesting.latest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class StackAutoRedSplines2plus1DoorMeep {

    //This is code for BackdropAutoBlue

    //coordinates for start position

    public static double startPoseX= -37.89893;
    public static double startPoseY= -62.90750;
    public static double startPoseAngle= Math.toRadians(270);

    public static Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    //coordinates for left spike position
    public static double spike1X = -38.1, spike1Y = -25.2;
    public static double spike1BackX = -34, spike1BackY = spike1Y;
    public static double spike1PreStackX = spike1BackX, spike1PreStackY = -22;

    //coordinates for middle spike position
    public static double spike2X = -38.3, spike2Y = -19.2, spike2Angle = Math.toRadians(330);
    public static double spike2BackX = -45, spike2BackY = -14;
    public static double spike2TurnAmount = Math.toRadians(180) - spike2Angle;

    //coordinates for right spike position
    public static double spike3PreX = -40, spike3PreY = -36;
    public static double spike3X = -33, spike3Y = -30;
    public static double spike3BackAmount = 8;
    public static double spike3PreStackX = -47, spike3PreStackY = -16, spike3PreStackAngle = Math.toRadians(181);

    //coordinates for stack and to backdrop
    public static double doorStackX = -60, doorStackY = -11.3;
    public static double doorStackForwardAmount = 4;
    public static double underDoorX = 23, underDoorY = doorStackY;

    //coordinates for backdrop positions
    public static double backdropMiddleX = 47;
    public static double backdropMiddleY = -35;
    public static double backdropMiddleAngle = Math.toRadians(180);
    public static double backdropBackAmount = 5;

    public static double backdropLeftStrafe = 4;
    public static double backdropRightStrafe = 4;

    //intake and outtake movement
    public static int outtakeEncoderTicksUp = 2500;
    public static double temporalMarkerTimeUp = 1.5;
    public static int outtakeEncoderTicksDown = 0;
    public static double temporalMarkerTimeDown = 1;

    //coordiantes for alt park
    public static double altParkForwardAmount = 4.5;
    public static double altParkX = backdropMiddleX - altParkForwardAmount, altParkY = 10;

    public static double casenum = 1;

    public static String myPosition;

    public static void main(String[] args) {
        MeepMeep meepmeep = new MeepMeep(500);

        RoadRunnerBotEntity myLeftBot = new DefaultBotBuilder(meepmeep)
                // We set this bot to be left
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startPose)
                                        .lineToLinearHeading(new Pose2d(spike1X, spike1Y, Math.toRadians(180)))
                                        .lineToConstantHeading(new Vector2d(spike1BackX, spike1BackY))

                                        .lineToConstantHeading(new Vector2d(spike1PreStackX, spike1PreStackY))
                                        .splineToConstantHeading(new Vector2d(doorStackX, doorStackY), Math.toRadians(180))
                                        .forward(doorStackForwardAmount)

                                        .setReversed(true)
                                        .lineToConstantHeading(new Vector2d(underDoorX, underDoorY))
                                        .splineTo(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY + backdropLeftStrafe), Math.toRadians(0))
                                        .forward(backdropBackAmount)

                                        // alt park
                                        .setReversed(false)
                                        .forward(altParkForwardAmount)
                                        .lineToConstantHeading(new Vector2d(altParkX, altParkY))

                                        .build()
                );

        RoadRunnerBotEntity myMiddleBot = new DefaultBotBuilder(meepmeep)
                // We set this bot to be right
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startPose)
                                        .setReversed(false)
                                        .lineToLinearHeading(new Pose2d(spike2X, spike2Y, spike2Angle))
                                        .lineToConstantHeading(new Vector2d(spike2BackX, spike2BackY))
//                                        .lineToSplineHeading(new Pose2d(-48.8, -12.9, Math.toRadians(180)))
                                        .turn(spike2TurnAmount)

//                                        .splineToConstantHeading(new Vector2d(-2 0, -11.3), Math.toRadians(180))
//                                        .splineToConstantHeading(new Vector2d(-50.2, -11.3), Math.toRadians(180))
                                        .lineToConstantHeading(new Vector2d(-60, -11.3))

                                        .setReversed(true)
                                        .lineToConstantHeading(new Vector2d(23, -11.3))
                                        .splineTo(new Vector2d(backdropMiddleX, backdropMiddleY), Math.toRadians(0))
                                        .forward(backdropBackAmount)

                                        // alt park
                                        .setReversed(false)
                                        .forward(altParkForwardAmount)
                                        .lineToConstantHeading(new Vector2d(altParkX, altParkY))

                                        .build()
                );

        RoadRunnerBotEntity myRightBot = new DefaultBotBuilder(meepmeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startPose)
                                        .setReversed(false)
                                        .lineToSplineHeading(new Pose2d(spike3PreX, spike3PreY, Math.toRadians(0)))
                                        .splineToConstantHeading(new Vector2d(spike3X, spike3Y), Math.toRadians(0))
                                        .setReversed(false)
                                        .back(spike3BackAmount)
                                        .lineToSplineHeading(new Pose2d(spike3PreStackX, spike3PreStackY, spike3PreStackAngle))
//                                        .lineToConstantHeading(new Vector2d(-34, -22))
                                        .splineToConstantHeading(new Vector2d(-60, -11.3), Math.toRadians(180))

                                        .setReversed(true)
                                        .lineToConstantHeading(new Vector2d(23, -11.3))
                                        .splineTo(new Vector2d(backdropMiddleX, backdropMiddleY - backdropRightStrafe), Math.toRadians(0))
                                        .forward(backdropBackAmount)

                                        // alt park
                                        .setReversed(false)
                                        .forward(altParkForwardAmount)
                                        .lineToConstantHeading(new Vector2d(altParkX, altParkY))

                                        .build()
                );

        RoadRunnerBotEntity myTestBot = new DefaultBotBuilder(meepmeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startPose)
//                                        .setReversed(true)
//                                        .splineTo(spike3Vector, Math.toRadians(0))
//                                        .back(spike3BackAmount,
//                                                SampleMecanumDrive.getVelocityConstraint(10, 2, 14),
//                                                SampleMecanumDrive.getAccelerationConstraint(50))
//
//                                        .splineToConstantHeading(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY - backdropRightStrafe), Math.toRadians(0))
//                                        .back(backdropBackAmount,
//                                                SampleMecanumDrive.getVelocityConstraint(10, 2, 14),
//                                                SampleMecanumDrive.getAccelerationConstraint(50))
//
//                                        //cycling
//                                        .setReversed(false)
//                                        .splineToConstantHeading(preDoorVector, Math.toRadians(180))
//                                        .splineToConstantHeading(underDoorVector, Math.toRadians(180))
//                                        .forward(doorStackForwardAmount,
//                                                SampleMecanumDrive.getVelocityConstraint(10, 2, 14),
//                                                SampleMecanumDrive.getAccelerationConstraint(50))
//                                        .setReversed(true)
//                                        .lineToConstantHeading(new Vector2d(backUnderDoorX, -12))
//                                        .splineToConstantHeading(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY + backdropLeftStrafe), Math.toRadians(0))
//                                        .back(backdropBackAmount,
//                                                SampleMecanumDrive.getVelocityConstraint(10, 2, 14),
//                                                SampleMecanumDrive.getAccelerationConstraint(50))
//
//                                        //parking in the corner:
////                                .setReversed(false)
////                                .forward(parkForwardAmount,
////                                        SampleMecanumDrive.getVelocityConstraint(10, 2, 14),
////                                        SampleMecanumDrive.getAccelerationConstraint(50))
////                                .splineToConstantHeading(preParkVector, Math.toRadians(0))
////                                .turn(Math.toRadians(-90))
////                                .lineToLinearHeading(parkPose)
//
//                                        // alt park
//                                        .setReversed(false)
//                                        .forward(altParkForwardAmount)
//                                        .splineTo(new Vector2d(backdropMiddleX + preAltParkXChange, preAltParkY), Math.toRadians(90))
//                                        .splineToConstantHeading(altParkVector, Math.toRadians(0))


                                        .lineToSplineHeading(new Pose2d(14.6, -33.2, Math.toRadians(180)))
                                        .forward(4)
                                        .lineToConstantHeading(new Vector2d(backdropMiddleX, backdropMiddleY + backdropLeftStrafe))
                                        .lineToConstantHeading(new Vector2d(32.1, -11.3))
                                        .lineToConstantHeading(new Vector2d(-53, -11.3))
                                        .forward(5)

                                        .lineToConstantHeading(new Vector2d(32.1, -11.3))
                                        .lineToConstantHeading(new Vector2d(backdropMiddleX, backdropMiddleY))

                                        .lineToConstantHeading(new Vector2d(36.1, -14.0))
                                        .lineToLinearHeading(new Pose2d(58, -11.7, Math.toRadians(90)))

                                        .build()
                );

        meepmeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(.95f)
                .addEntity(myLeftBot)
                .addEntity(myMiddleBot)
                .addEntity(myRightBot)
//                .addEntity(myTestBot)
                .start();
    }
}