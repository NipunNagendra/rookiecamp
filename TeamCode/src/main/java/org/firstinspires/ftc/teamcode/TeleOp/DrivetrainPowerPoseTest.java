package org.firstinspires.ftc.teamcode.TeleOp;

import static com.sun.tools.doclint.Entity.pi;

import android.annotation.SuppressLint;
import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.CustomMecanumKinematics;
import org.firstinspires.ftc.teamcode.libs.Manipulators;
import org.firstinspires.ftc.teamcode.libs.Movement;
import org.firstinspires.ftc.teamcode.libs.SensorLibrary;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="DrivetrainPowerPoseTest", group="TeleOp")
public class DrivetrainPowerPoseTest extends OpMode {
    Movement move;

    Manipulators manip;

    DistanceSensor ds;

    public static double threshold =15;
    SensorLibrary sensor;
    //RevColorSensorV3 cs;
    //DistanceSensor ds;
    String moveType = "robot";
    boolean outtakeServoStatus = false;
    double[] motorPower = {0, 0, 0, 0};
    double multiplier = 1;
    Pair<float[], String> hsv_color;

    IMU imu;

    public static double heading;

    public static double targetAngle;

    public static String lockStatus;

    public static FtcDashboard dashboard;
    public static Telemetry dashboardTelemetry;


    public void init() {
        manip = new Manipulators(hardwareMap);
//        manip.droneServo.setPower(0);
        move = new Movement(hardwareMap);
//        sensor = new SensorLibrary(hardwareMap);
        //cs = hardwareMap.get(RevColorSensorV3.class, "cs");
        gamepad1.setLedColor(0, 0, 256, 100000);
        telemetry.addData("init", "completed");
        telemetry.update();
        // imu for field oriented drive
        imu = hardwareMap.get(IMU.class, "imu");
        //TODO: Change for acc roibot
         dashboard = FtcDashboard.getInstance();
         dashboardTelemetry = dashboard.getTelemetry();
        // season bot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(new Orientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZXY,
                AngleUnit.DEGREES,
                -90,
                90,
                -33,
                0
        )

        ));

        // trollbot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
//        ));

        imu.initialize(parameters);
        imu.resetYaw();
        lockStatus = "unlocked";


    }


    @SuppressLint("SuspiciousIndentation")
    @Override
    public void loop()
    {


        double leftY;
        double leftX;
        double rightX;



        if (move.isPressed("options1",gamepad1.options)) {
            imu.resetYaw();
        }


        heading = move.drive.getExternalHeading();

        if (Math.abs(gamepad1.left_stick_y)  > 0.1 ||
                Math.abs(gamepad1.left_stick_x)  > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1) {

            leftY = gamepad1.left_stick_y * multiplier;
            leftX = (gamepad1.left_stick_x * multiplier);
            rightX = (gamepad1.right_stick_x * multiplier);

            move.fieldDrive(leftX, leftY, rightX, heading, 0);
        }
        else {
            move.fieldDrive(0,0, 0, heading, 0);
        }

        dashboardTelemetry.addData("x_given", Movement.vel.getX());
        dashboardTelemetry.addData("y_given", Movement.vel.getY());
        dashboardTelemetry.addData("heading_given", Movement.vel.getHeading());

        dashboardTelemetry.addData("x_actual", CustomMecanumKinematics.forwardKinematicModel(move.getWheelPowers()).getX());
        dashboardTelemetry.addData("y_actual", CustomMecanumKinematics.forwardKinematicModel(move.getWheelPowers()).getY());
        dashboardTelemetry.addData("heading_actual", CustomMecanumKinematics.forwardKinematicModel(move.getWheelPowers()).getHeading());
        dashboardTelemetry.update();

        telemetry.update();
    }}
