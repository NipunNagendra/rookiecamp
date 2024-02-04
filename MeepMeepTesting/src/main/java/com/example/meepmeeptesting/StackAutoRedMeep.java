
package com.example.meepmeeptesting;

        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.noahbres.meepmeep.MeepMeep;
        import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
        import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class StackAutoRedMeep {
    //coordinates for starting position (0, 0, 0)
    public static double startPoseX= -38.15845302224215;
    public static double startPoseY= -65.13672263931143;
    public static double startPoseAngle= Math.toRadians(270);

    Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseAngle);

    Pose2d posEstimate;

    //coordinates for left spike position
    public static double spike1X = -41.64633638294297;
    public static double spike1Y = -32.1247700133697;
    public static double spike1Angle = Math.toRadians(180);

    //coordinates for middle spike position
    public static double spike2X =  -37.812297556497846;
    public static double spike2Y = -27.023006373520104;
    public static double spike2Angle = Math.toRadians(90);

    //coordinates for right spike position
//    public static double spike3X = -34.26642694740993;
//    public static double spike3Y = 29.54644728121096;
//    public static double spike3Angle = Math.toRadians(180);
    public static double moveBackwards3 = 31;
    public static double moveForward3 = 11;
    public static double turn3 = 90;

    public static double preTrussX = -38.15845302224215;
    public static double trussX = 15;
    public static double trussY = -55.93672263931143;
    public static double trussAngle = Math.toRadians(180);
    public static double backdropMiddleX = 46;
    public static double backdropMiddleY = -38;
    public static double backdropMiddleAngle = trussAngle;
    public static double backdropLeftStrafe = 8;
    public static double backdropRightStrafe = 8;

    public static double casenum=1;


    public static String myPosition;

    public static void main(String[] args) {
        MeepMeep meepmeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepmeep)
                .setConstraints(35.86228895377674, 35.86228895377674, 2.4242626190185548, Math.toRadians(214.79), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseAngle))
                                .back(moveBackwards3)
                                .turn(Math.toRadians(turn3))
                                .forward(moveForward3)
                                .back(10)
                                .turn(Math.toRadians(-180))
                                .lineToLinearHeading(new Pose2d(preTrussX, trussY, trussAngle))
                                .lineToLinearHeading(new Pose2d(trussX, trussY, trussAngle))
                                .lineToLinearHeading(new Pose2d(backdropMiddleX, backdropMiddleY, backdropMiddleAngle))
                                .strafeLeft(backdropRightStrafe)
                                .build()


                );


        meepmeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(.95f)
                .addEntity(myBot)
                .start();
    }



}