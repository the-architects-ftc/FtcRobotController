package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
//import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="Teleop_live", group="Exercises")
//@Disabled
public class Teleop_live extends LinearOpMode {
    BHI260IMU imu;
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
    Servo s3 = null;
    float leftY, rightY,leftX,rightX;
    boolean flag_correction = true;
    boolean intake_constant = false;
    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        // Variable Initialization
        BHI260IMU.Parameters myIMUParameters;
        Orientation myRobotOrientation;

        // IMU in the control hub
        imu = hardwareMap.get(BHI260IMU.class,"imu");

        // Start imu initialization
        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,RevHubOrientationOnRobot.UsbFacingDirection.UP )
        );
        imu.initialize(myIMUParameters);
        imu.resetYaw();
        telemetry.addData("Gyro Status", "Initialized");
        telemetry.update();

        // wheel motors
        bl = hardwareMap.get(DcMotor.class, "LB");
        fl = hardwareMap.get(DcMotor.class, "LF");
        fr = hardwareMap.get(DcMotor.class, "RF");
        br = hardwareMap.get(DcMotor.class, "RB");
        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m0 = hardwareMap.get(DcMotor.class, "M0");
        m1 = hardwareMap.get(DcMotor.class, "M1");
        m2 = hardwareMap.get(DcMotor.class, "M2");
        m3 = hardwareMap.get(DcMotor.class, "M3");
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s1.setDirection(Servo.Direction.FORWARD);
        s2.setDirection(Servo.Direction.FORWARD);

        // Initial settings
        bl.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        fr.setPower(0);

        // Start opMode
        telemetry.addData("Mode", "waiting");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            leftY = gamepad1.left_stick_y * -1;
            rightY = gamepad1.right_stick_y * -1;
//            leftX = gamepad1.left_stick_x * -1;
//            rightX = gamepad1.right_stick_x * -1;


            if (gamepad1.b) {
                m2.setPower(-0.5);
                m3.setPower(0.5);
            } else {
                m2.setPower(0);
                m3.setPower(0);
            }

            if (gamepad1.a) {
                m2.setPower(0.5);
                m3.setPower(-0.5);
            } else {
                m2.setPower(0);
                m3.setPower(0);

            }

            if (gamepad1.y) {
                s3.setPosition(0.5);

            }

            if (gamepad1.left_bumper) {
                s1.setPosition(0.4);
            }
            if (gamepad1.right_bumper) {
                s1.setPosition(0.3);
            }
            if (gamepad1.x) {
                intake_constant = !intake_constant;
                telemetry.addData("intake",intake_constant);
                telemetry.update();
                sleep(500);

                if (intake_constant == true) {
                    m0.setPower(-1);
                    m1.setPower(1);
                } else {
                    m0.setPower(0);
                    m1.setPower(0);
                }
            }



            if (gamepad1.left_trigger > 0.5) {
                s2.setPosition(0);
            }
            if (gamepad1.right_trigger > 0.5) {
                s2.setPosition(0.28);
            }
            //Forwards/Backwards
            if (leftY != 0 && (rightY == 0)) {
                double THRESH_WM_POWER = 0.8; // max abs wheel power
                myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                double correction = myRobotOrientation.thirdAngle / 180.0;
                correction = (10.0 * correction * Math.abs(leftY) / THRESH_WM_POWER);
                if (flag_correction == false) {
                    correction = 0;

                }

                telemetry.addData("first Angle", myRobotOrientation.firstAngle);
                telemetry.addData("second Angle", myRobotOrientation.secondAngle);
                telemetry.addData("third Angle", myRobotOrientation.thirdAngle);
                telemetry.addData("correction", correction);
                telemetry.addData("leftY", leftY);
                telemetry.addData("fl Enc Count", fl.getCurrentPosition());
                telemetry.addData("bl Enc Count", bl.getCurrentPosition());
                telemetry.addData("fr Enc Count", fr.getCurrentPosition());
                telemetry.addData("br Enc Count", br.getCurrentPosition());
                telemetry.update();
                telemetry.update();

                //correction = 0;
                fl.setPower(Range.clip(leftY + correction, -THRESH_WM_POWER, THRESH_WM_POWER));
                bl.setPower(Range.clip(leftY + correction, -THRESH_WM_POWER, THRESH_WM_POWER));
                fr.setPower(Range.clip(leftY - correction, -THRESH_WM_POWER, THRESH_WM_POWER));
                br.setPower(Range.clip(leftY - correction, -THRESH_WM_POWER, THRESH_WM_POWER));
            }

            // DPad - down
            if (gamepad1.dpad_down) {
                imu.resetYaw();
                bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fl.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
                fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                br.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

                bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            //DPad - up
            if (gamepad1.dpad_up) {
                flag_correction = !flag_correction;
                boolean flag_2 = flag_correction;
                flag_correction = flag_2;
                sleep(1000);
                telemetry.addData("Correction status", flag_correction);
                telemetry.update();
            }

            //sideways:
            if (gamepad1.dpad_left || gamepad1.dpad_right) {
                double directionFactor = 1;
                if (gamepad1.dpad_right) {
                    directionFactor = -1;
                }
                double THRESH_WM_POWER_SIDEWAYS = 0.8;
                double frEC = fr.getCurrentPosition();
                double blEC = bl.getCurrentPosition();
                double flEC = fl.getCurrentPosition();
                double brEC = br.getCurrentPosition();
                double frCorr = 1;
                double blCorr = 1;
                double flCorr = 1;
                double brCorr = 1;
                if (frEC != 0 && (flag_correction == true)) {
                    frEC = Math.abs(frEC);
                    double refEC = frEC;
                    blEC = Math.abs(blEC);
                    refEC = Math.min(refEC, blEC);
                    flEC = Math.abs(flEC);
                    refEC = Math.min(refEC, flEC);
                    brEC = Math.abs(brEC);
                    refEC = Math.min(refEC, brEC);
                    if (refEC == 0) {
                        refEC = 1;
                    }
                    frCorr = refEC / frEC;
                    blCorr = refEC / blEC;
                    flCorr = refEC / flEC;
                    brCorr = refEC / brEC;
                }
                double flPow = directionFactor * -THRESH_WM_POWER_SIDEWAYS * flCorr;
                double blPow = directionFactor * THRESH_WM_POWER_SIDEWAYS * blCorr;
                double frPow = directionFactor * THRESH_WM_POWER_SIDEWAYS * frCorr;
                double brPow = directionFactor * -THRESH_WM_POWER_SIDEWAYS * brCorr;
                fl.setPower(flPow);
                bl.setPower(blPow);
                fr.setPower(frPow);
                br.setPower(brPow);

                telemetry.addData("fl Pow", flPow);
                telemetry.addData("bl Pow", blPow);
                telemetry.addData("fr Pow", frPow);
                telemetry.addData("br Pow", brPow);
                telemetry.addData("fl Enc Count", fl.getCurrentPosition());
                telemetry.addData("bl Enc Count", bl.getCurrentPosition());
                telemetry.addData("fr Enc Count", fr.getCurrentPosition());
                telemetry.addData("br Enc Count", br.getCurrentPosition());
                telemetry.update();
            }


            if (leftY == 0) {
                //turning
                double THRESH_WM_POWER_FORTURN = 0.8;
                fl.setPower(Range.clip(rightY * 0.6, -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN));
                bl.setPower(Range.clip(rightY, -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN));
                fr.setPower(Range.clip(-rightY * 0.6, -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN));
                br.setPower(Range.clip(-rightY, -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN));

               imu.resetYaw();
                idle();
                }
            }

    }


}