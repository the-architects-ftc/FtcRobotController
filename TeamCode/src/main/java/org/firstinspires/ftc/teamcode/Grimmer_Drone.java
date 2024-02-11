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


@TeleOp(name="Grimmer_Drone", group="Exercises")
//@Disabled
public class Grimmer_Drone extends LinearOpMode {
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
    float left1Y, right1Y,left1X,right1X;
    float left2Y, right2Y, left2X, right2X;
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
        s3.setDirection(Servo.Direction.REVERSE);

        // Initial settings
        bl.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        fr.setPower(0);

        // Start opMode
        telemetry.addData("Mode", "waiting");
        telemetry.update();
        s2.setPosition(0.147);
        //s2.(0.147) is the init ( flat position )
        s1.setPosition(0.2);
        waitForStart();
        while (opModeIsActive()) {

            // initiate the left and right variables
            left1Y = gamepad1.left_stick_y * -1;
            left1X = gamepad1.left_stick_x;
            right1Y = gamepad1.right_stick_y * -1;
            right1X = gamepad1.right_stick_x;

            left2Y = gamepad2.left_stick_y * -1;
            left2X = gamepad2.left_stick_x;
            right2Y = gamepad2.right_stick_y * -1;
            right2X = gamepad2.right_stick_x;



            // B for gamepad 1 and 2
            if (gamepad1.b) {
                m2.setPower(-1);
                m3.setPower(1);
            } else {
                m2.setPower(0);
                m3.setPower(0);
            }
            if (gamepad2.b) {
                m2.setPower(-1);
                m3.setPower(1);
            } else {
                m2.setPower(0);
                m3.setPower(0);
            }
            // A for gamepad 1 and 2
            if (gamepad1.a) {

                m2.setPower(1);
                m3.setPower(-1);
            } else {
                m2.setPower(0);
                m3.setPower(0);

            }
            if (gamepad2.a) {

                m2.setPower(1);
                m3.setPower(-1);
            } else {
                m2.setPower(0);
                m3.setPower(0);

            }
            // Y for gamepad 1 and 2
            if (gamepad1.y) {
                s3.setPosition(0);

            }
            if (gamepad2.y) {
                s3.setPosition(0);

            }

        }

    }


}