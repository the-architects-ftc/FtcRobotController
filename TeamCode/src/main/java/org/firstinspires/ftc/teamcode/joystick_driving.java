package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="joystick_driving", group="Exercises")
//@Disabled
public class joystick_driving extends LinearOpMode {
    DcMotor bl = null;
    DcMotor fr = null;
    DcMotor br = null;

    DcMotor fl = null;
    float leftY, rightY,leftX,rightX;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        bl = hardwareMap.get(DcMotor.class, "LB");
        fl = hardwareMap.get(DcMotor.class, "LF");
        fr = hardwareMap.get(DcMotor.class, "RF");
        br = hardwareMap.get(DcMotor.class, "RB");

        bl.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();
waitForStart();
    while (opModeIsActive()) {
        leftY = gamepad1.left_stick_y * -1;
        rightY = gamepad1.right_stick_y * -1;
        leftX = gamepad1.left_stick_x * -1;
        rightX = gamepad1.right_stick_x * -1;


        //Forwards/Backwards
        fl.setPower(Range.clip(leftY, -1.0, 1.0));
        bl.setPower(Range.clip(leftY, -1.0, 1.0));
        fr.setPower(Range.clip(leftY, -1.0, 1.0));
        br.setPower(Range.clip(leftY, -1.0, 1.0));

        //sideways:
        fl.setPower(Range.clip(leftX, -1.0, 1.0));
        bl.setPower(Range.clip(leftX, 1.0, -1.0));
        fr.setPower(Range.clip(leftX, -1.0, 1.0));
        br.setPower(Range.clip(leftX, -1.0, 1.0));

        //Turning
        fl.setPower(Range.clip(rightX, -1.0, 1.0));
        bl.setPower(Range.clip(rightX, 1.0, -1.0));
        fr.setPower(Range.clip(rightX, 1.0, -1.0));
        br.setPower(Range.clip(rightX, 1.0, -1.0));

        fl.setPower(Range.clip(rightY, -1.0, 1.0));
        bl.setPower(Range.clip(rightY, 1.0, -1.0));
        fr.setPower(Range.clip(rightY, 1.0, -1.0));
        br.setPower(Range.clip(rightY, 1.0, -1.0));


        telemetry.addData("Mode", "running");
        telemetry.addData("direction", bl.getPower());
        telemetry.addData("direction", br.getPower());
        telemetry.addData("direction", fl.getPower());
        telemetry.addData("direction", fr.getPower());

        idle();

    }

    }


}