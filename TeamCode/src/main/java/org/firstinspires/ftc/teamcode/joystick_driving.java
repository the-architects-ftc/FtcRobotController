package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="joystick_driving", group="Exercises")
//@Disabled
public class joystick_driving extends LinearOpMode {
    DcMotor bl = null;
    DcMotor fr = null;
    DcMotor br = null;

    DcMotor fl = null;
    float leftY, rightY;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        bl = hardwareMap.get(DcMotor.class, "LB");
        fl = hardwareMap.get(DcMotor.class, "LF");
        fr = hardwareMap.get(DcMotor.class, "RF");
        br = hardwareMap.get(DcMotor.class, "RB");

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Mode", "waiting");
        telemetry.update();


        leftY = gamepad1.left_stick_y * -1;
        rightY = gamepad1.right_stick_y * -1;


        fl.setPower(Range.clip(leftY, -1.0, 1.0));
        bl.setPower(Range.clip(leftY, -1.0, 1.0));

        fr.setPower(Range.clip(rightY, -1.0, 1.0));
        br.setPower(Range.clip(rightY, -1.0, 1.0));


        telemetry.addData("Mode", "running");
        telemetry.addData("sticks", "  left=" + leftY + "  right=" + rightY);
    }

    ;
}