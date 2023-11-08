package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.libs.Movement;

@Config
@Autonomous(name = "ParkAutoRight", group = "Autonomous")

public class ParkAutoRed extends LinearOpMode{
    Movement move;

    @Override
    public void runOpMode() throws InterruptedException{
        move = new Movement(hardwareMap);
        telemetry.addLine("Init Done");

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            move.strafeRight(1);
            sleep(1000);
            move.kill();
        }

    }}
