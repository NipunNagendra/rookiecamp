package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.libs.Movement;

import java.util.HashMap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="RAMPE", group="TeleOp")
public class RAMPE extends OpMode {

    Movement move;
    public HashMap<String, Boolean> buttons = new HashMap<String, Boolean>();
    double[] motorPower = {0, 0, 0, 0};

    double FRM = .5;
    double FLM = .5;
    double BRM = .5;
    double BLM = .5;


    double leftY;
    double leftX;
    double rightX;

    public void init() {
        move = new Movement(hardwareMap);

        //positions = manip.setPositions();
        telemetry.addData("init", "completed");
        telemetry.update();
    }

    public boolean isPressed(String name, boolean button){
        boolean output = false;

        //If the hashmap doesn't already contain the key
        if (!buttons.containsKey(name)) {
            buttons.put(name, false);
        }

        boolean buttonWas = buttons.get(name);
        if (button != buttonWas && button == true){
            output = true;
        }

        buttons.put(name, button);

        return output;
    }

    @Override
    public void loop() {

        if (isPressed("y2", gamepad1.y)) {
            FLM += 0.1;
        }
        if (isPressed("b2", gamepad1.b)) {
            FRM += 0.1;
        }
        if (isPressed("x2", gamepad1.x)) {
            BLM += 0.1;
        }
        if (isPressed("a2", gamepad1.a)) {
            BRM += 0.1;
        }

        if (isPressed("dpad_up2", gamepad1.dpad_up)) {
            FLM -= 0.1;
        }
        if (isPressed("dpad_right2", gamepad1.dpad_right)) {
            FRM -= 0.1;
        }
        if (isPressed("dpad_left2", gamepad1.dpad_left)) {
            BLM -= 0.1;
        }
        if (isPressed("dpad_down2", gamepad1.dpad_down)) {
            BRM -= 0.1;
        }



        if (Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1) {

            leftY = gamepad1.left_stick_y;
            leftX = -1 * (gamepad1.left_stick_x);
            rightX = (-gamepad1.right_stick_x);

            motorPower = move.holonomicDrive(leftX, leftY, rightX);

        } else {
            motorPower = move.holonomicDrive(0, 0, 0);
        }


        move.setPowers(motorPower[0]*FRM, motorPower[1]*FLM, motorPower[2]*BRM, motorPower[3]*BLM);

        telemetry.addData("FR power: ", FRM);
        telemetry.addData("FL power: ", FLM);
        telemetry.addData("BR power: ", BRM);
        telemetry.addData("BL power: ", BLM);



        telemetry.update();
    }
}
