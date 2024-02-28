
package com.example.meepmeeptesting;

        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.noahbres.meepmeep.MeepMeep;
        import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
        import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class StackAutoRedMeep {
    //coordinates for starting position (0, 0, 0)
    public static double startPoseX= -35.19893;
    public static double startPoseY= -62.90750;
    public static double startPoseAngle= Math.toRadians(270);

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    //coordinates for left spike position
    public static double spike1X = -37.85502;
    public static double spike1Y = -31.03817;
    public static double spike1Angle = Math.toRadians(180);

    //coordinates for middle spike position
    public static double spike2X =  -35.76226;
    public static double spike2Y = -30.03102;
    public static double spike2Angle = Math.toRadians(90);

    //coordinates for right spike position
//    public static double spike3X = -34.26642694740993;
//    public static double spike3Y = 29.54644728121096;
//    public static double spike3Angle = Math.toRadians(180);
    public static double moveBackwards3 = 31;
    public static double moveForward3 = 12;
    public static double turn3 = 90;

    public static double preTrussX = -33.04398;
    public static double trussX = 35.26448;
    public static double dangerPathX = -15;
    public static double trussY = -57.10612;
    public static double trussAngle = Math.toRadians(180);

    public static double goingDirectlyUnderTruss = 15;
    public static double betweenTruss = 35;
    public static double exitDoor = 15;

    public static double preBackdropX = 37.08193;
    public static double preBackdropY = -30.32037;
    public static double preBackdropAngle = Math.toRadians(180);

    public static double backdropMiddleX = 49.18600;
    public static double backdropMiddleY = -30.70411;
    public static double backdropMiddleAngle = trussAngle;
    public static double backdropLeftStrafe = 5;
    public static double backdropRightStrafe = 5;
    public static double strafeValue = 0;

    public static double preParkY = -54.13497;
    public static double goingIntoPark = 14;
    public static double temporalMarkerTime = 1.5;

    public static double temporalMarkerTimeAlternate = 4;

    public static double outFromBackdrop = 6;

    public static int outtakeEncoderTicksUp = 1800;
    public static int outtakeEncoderTicksDown = 0;
    public static double temporalMarkerTimeUp = 1.5;
    public static double temporalMarkerTimeDown = 0.5;

    public static double casenum=1;



    public static String myPosition;

    public static void main(String[] args) {
        MeepMeep meepmeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepmeep)
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(preTrussX, trussY, trussAngle))
//                                .back(moveBackwards3)
//                                .turn(Math.toRadians(turn3))
//                                .forward(moveForward3)
//                                .back(10)
//                                .turn(Math.toRadians(-180))
//                                .lineToLinearHeading(new Pose2d(preTrussX, trussY, trussAngle))
//                                .lineToLinearHeading(new Pose2d(trussX, trussY, trussAngle))
//                                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
//                                .strafeLeft(backdropRightStrafe)
//                                .build()
                                .lineToLinearHeading(new Pose2d(trussX, trussY, trussAngle))
                                .lineToLinearHeading(new Pose2d(preBackdropX, preBackdropY, preBackdropAngle))
                                .lineToLinearHeading(
                                        new Pose2d(backdropMiddleX, backdropMiddleY + strafeValue, backdropMiddleAngle))
                                .build()


                );


        meepmeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(.95f)
                .addEntity(myBot)
                .start();
    }



}