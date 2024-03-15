package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class BackdropAutoRedSplinesMeep {

    //This is code for BackdropAutoBlue

    //coordinates for start position

    public static double startPoseX = 14.65845302224215;
    public static double startPoseY = -65.13672263931143;
    public static double startPoseAngle = Math.toRadians(270);

    public static Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    //coordinates for left spike position
    public static double spike1ForwardAmount = 5;
    public static Pose2d preSpike1Pose = new Pose2d(19.2, -33.8 - 4, Math.toRadians(180));
    public static Vector2d spike1Vector = new Vector2d(13.25 - spike1ForwardAmount, -33.8);

    //coordinates for middle spike position
    public static Vector2d preSpike2Vector = new Vector2d(12.3, -42.3);
    public static Vector2d spike2Vector = new Vector2d(21.8, -24);

    //coordinates for right spike position
    public static Vector2d spike3Vector = new Vector2d(31.5, -31);
    public static double spike3BackAmount = 3;

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

    //coordiantes for park in corner
    public static double parkForwardAmount = 3;
    public static Vector2d preParkVector = new Vector2d(44, -56);
    public static Pose2d parkPose = new Pose2d(58.7, -59, Math.toRadians(90));

    //coordiantes for alt park
    public static double prePreAltParkXChange = 8;
    public static double prePreAltParkY = -34;
    public static double preAltParkXChange = prePreAltParkXChange;
    public static double preAltParkY = -20;
    public static Vector2d altParkVector = new Vector2d(58.7, -11.3);

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
                                .setReversed(true)
                                .back(1)
                                .splineToSplineHeading(preSpike1Pose, Math.toRadians(90))
                                .splineToConstantHeading(spike1Vector, Math.toRadians(0))
                                .back(spike1ForwardAmount,
                                        SampleMecanumDrive.getVelocityConstraint(10, 2, 14),
                                        SampleMecanumDrive.getAccelerationConstraint(50))

                                .splineToConstantHeading(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY + backdropLeftStrafe), Math.toRadians(0))
                                .back(backdropBackAmount,
                                        SampleMecanumDrive.getVelocityConstraint(10, 2, 14),
                                        SampleMecanumDrive.getAccelerationConstraint(50))

                                //parking in the corner:
                                .setReversed(false)
                                .forward(parkForwardAmount,
                                        SampleMecanumDrive.getVelocityConstraint(10, 2, 14),
                                        SampleMecanumDrive.getAccelerationConstraint(50))
                                .splineToConstantHeading(preParkVector, Math.toRadians(0))
                                .turn(Math.toRadians(-90))
                                .lineToLinearHeading(parkPose)

                                // alt park
//                                .setReversed(false)
//                                .forward(4.5)
//                                .splineTo(new Vector2d(backdropMiddleX - 10, -20), Math.toRadians(90))
//                                .splineToConstantHeading(new Vector2d(58.7, -11.3), Math.toRadians(0))

                                .build()
                );

        RoadRunnerBotEntity myMiddleBot = new DefaultBotBuilder(meepmeep)
                // We set this bot to be right
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .setReversed(true)
//                                .back(12)
                                .splineTo(preSpike2Vector, Math.toRadians(90))
                                .splineTo(spike2Vector, Math.toRadians(0))

                                .splineTo(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY), Math.toRadians(0))
                                .back(backdropBackAmount,
                                        SampleMecanumDrive.getVelocityConstraint(10, 2, 14),
                                        SampleMecanumDrive.getAccelerationConstraint(50))

                                //parking in the corner:
                                .setReversed(false)
                                .forward(parkForwardAmount,
                                        SampleMecanumDrive.getVelocityConstraint(10, 2, 14),
                                        SampleMecanumDrive.getAccelerationConstraint(50))
                                .splineToConstantHeading(preParkVector, Math.toRadians(0))
                                .turn(Math.toRadians(-90))
                                .lineToLinearHeading(parkPose)

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
                                drive.trajectorySequenceBuilder(startPose)
                                        .setReversed(true)
                                        .splineTo(spike3Vector, Math.toRadians(0))
                                        .back(spike3BackAmount,
                                                SampleMecanumDrive.getVelocityConstraint(10, 2, 14),
                                                SampleMecanumDrive.getAccelerationConstraint(50))

                                        .splineToConstantHeading(new Vector2d(backdropMiddleX - backdropBackAmount, backdropMiddleY - backdropRightStrafe), Math.toRadians(0))
                                        .back(backdropBackAmount,
                                                SampleMecanumDrive.getVelocityConstraint(10, 2, 14),
                                                SampleMecanumDrive.getAccelerationConstraint(50))

                                        //parking in the corner:
//                                .setReversed(false)
//                                .forward(3)
//                                .splineToConstantHeading(new Vector2d(44, -56), Math.toRadians(0))
//                                .turn(Math.toRadians(-90))
//                                .lineToLinearHeading(new Pose2d(58.7, -60, Math.toRadians(90)))

                                        // alt park
                                        .setReversed(false)
                                        .forward(4,
                                                SampleMecanumDrive.getVelocityConstraint(10, 2, 14),
                                                SampleMecanumDrive.getAccelerationConstraint(50))
                                        .splineTo(new Vector2d(backdropMiddleX - prePreAltParkXChange, prePreAltParkY), Math.toRadians(90))
                                        .lineToLinearHeading(new Pose2d(backdropMiddleX - preAltParkXChange, preAltParkY, Math.toRadians(90)))
                                        .splineToConstantHeading(altParkVector, Math.toRadians(0))

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