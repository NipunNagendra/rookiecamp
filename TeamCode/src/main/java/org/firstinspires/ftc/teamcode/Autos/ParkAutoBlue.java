package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libs.Movement;

@Config
@Autonomous(name = "ParkAutoBlue", group = "Autonomous")

public class ParkAutoBlue extends LinearOpMode{
    Movement move;

    @Override
    public void runOpMode() throws InterruptedException{
        move = new Movement(hardwareMap);
        telemetry.addLine("Init Done");

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            move.strafeLeft(1);
            sleep(1000);
            move.kill();
            break;
        }

    }}
