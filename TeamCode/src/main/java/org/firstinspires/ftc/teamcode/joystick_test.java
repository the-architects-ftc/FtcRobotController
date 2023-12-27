package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="Arm_joystick", group="Exercises")
//@Disabled
public class joystick_test extends LinearOpMode {
    DcMotor bl = null;
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor br = null;
    DcMotor m0 = null;
    DcMotor m1 = null;
    DcMotor m2 = null;
    DcMotor m3 = null;
    Servo s1 = null;
    Servo s2 = null;
    float leftY, rightY,leftX,rightX;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        bl = hardwareMap.get(DcMotor.class, "LB");
        fl = hardwareMap.get(DcMotor.class, "LF");
        fr = hardwareMap.get(DcMotor.class, "RF");
        br = hardwareMap.get(DcMotor.class, "RB");
        m0 = hardwareMap.get(DcMotor.class, "M0");
        m1 = hardwareMap.get(DcMotor.class, "M1");
        m2 = hardwareMap.get(DcMotor.class, "M2");
        m3 = hardwareMap.get(DcMotor.class, "M3");
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");


        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        s1.setDirection(Servo.Direction.FORWARD);
        s2.setDirection(Servo.Direction.FORWARD);
        bl.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        fr.setPower(0);

        telemetry.addData("Mode", "waiting");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            leftY = gamepad1.left_stick_y * -1;
            rightY = gamepad1.right_stick_y * -1;
            leftX = gamepad1.left_stick_x * -1;
            rightX = gamepad1.right_stick_x * -1;

            if (gamepad1.b) {
                m2.setPower(-0.5);
                m3.setPower(0.5);
            }
            else {
                m2.setPower(0);
                m3.setPower(0);
            }

            if (gamepad1.a) {
                m2.setPower(0.5);
                m3.setPower(-0.5);
            }
            else {
                m2.setPower(0);
                m3.setPower(0);

            }


            if(gamepad1.left_bumper){
                s1.setPosition(0.4);
            }

            if(gamepad1.right_bumper){
                s1.setPosition(0.3);
            }




            if(gamepad1.x){
                m0.setPower(1);
                m1.setPower(-1);
            }
            else {
                m0.setPower(0);
                m1.setPower(0);
            }


            if(gamepad1.left_trigger > 0.5) {
                s2.setPosition(0);
            }

            if(gamepad1.right_trigger > 0.5)
            {
                s2.setPosition(0.28);
            }

            //Forwards/Backwards
            fl.setPower(Range.clip(leftY, -0.3, 0.3));
            bl.setPower(Range.clip(leftY, -0.3, 0.3));
            fr.setPower(Range.clip(leftY, -0.3, 0.3));
            br.setPower(Range.clip(leftY, -0.3, 0.3));

            sideways:
//            fl.setPower(Range.clip(leftX, -1.0, 1.0));
//            bl.setPower(Range.clip(-leftX, -1.0, 1.0));
//            fr.setPower(Range.clip(-leftX, -1.0, 1.0));
//            br.setPower(Range.clip(leftX, -1.0, 1.0));
//
//            //turning
            fl.setPower(Range.clip(rightX, -1.0, 0.5));
            bl.setPower(Range.clip(rightX, -1.0, 1.0));
            fr.setPower(Range.clip(-rightX, -1.0, 0.5));
            br.setPower(Range.clip(-rightX, -1.0, 1.0));

            fl.setPower(Range.clip(rightY, -1.0, 0.5));
            bl.setPower(Range.clip(rightY, -1.0, 1.0));
            fr.setPower(Range.clip(-rightY, -1.0, 0.5));
            br.setPower(Range.clip(-rightY, -1.0, 1.0));



            //Forwards/Backwards
            idle();

        }

    }


}