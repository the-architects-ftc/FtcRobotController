package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommonUtil;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class CommonUtil extends LinearOpMode {

    Orientation myRobotOrientation;

    double ENC2DIST = 4593.0/102.0; //2000.0/48.0; // FW/BW
    double ENC2DIST_SIDEWAYS = 2911.0/57.0;
    ElapsedTime timer = new ElapsedTime();

    //imu init
    BHI260IMU imu;
    BHI260IMU.Parameters myIMUParameters;



    YawPitchRollAngles robotOrientation;

    //motor / servo init
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


    //All Our functions!

    // Initialize
    public void initialize(HardwareMap hardwareMap){

        telemetry.addData("at","initialize");
        telemetry.update();

        // map imu
        imu = hardwareMap.get(BHI260IMU.class,"imu");
        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,RevHubOrientationOnRobot.UsbFacingDirection.UP )
        );
        imu.initialize(myIMUParameters);
        imu.resetYaw();
        // Start imu initialization
        telemetry.addData("Gyro Status", "Initialized");
        telemetry.update();
        // map motors
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
        s3 = hardwareMap.get(Servo.class, "s3");
        s1.setDirection(Servo.Direction.FORWARD);
        s2.setDirection(Servo.Direction.FORWARD);
    }


    // Set motor directions
    public void setMotorOrientation()
    {
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

    }

    //reset encoder counts
    public void resetMotorEncoderCounts()
    {
        // Reset encoder counts kept by motors
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        telemetry.addData("Encoder", "Count Reset");  // telemetry: Mode Waiting
        telemetry.update();

    }

    //motor power 0
    public void setMotorToZeroPower()
    {
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

    public void setZeroPowerBehavior(){

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double PID_Turn (double targetAngle, double currentAngle, String minPower) {
        double sign = 1;
        double power = (targetAngle - currentAngle) * 0.05;
        if (minPower.equalsIgnoreCase("on")&& (power != 0)) {
            sign = Math.signum(power);
            power = Math.max(Math.abs(power), 0.1);
            power = power*sign;
        }
        return power;
    }

    public double PID_FB (double targetEC, double currentEC)
    {
        double power = (targetEC -currentEC)*0.0003;
        if (power < 0.3){
            power = 0.3;
        }
        return power;

    }
    //move forwards with gyro
    public int moveForward_wDistance_wGyro(double DistanceAbsIn,double Mpower)
    {

        double currZAngle = 0;
        int currEncoderCount = 0;
        double encoderAbsCounts = ENC2DIST*DistanceAbsIn;
        telemetry.addData("EC Target", encoderAbsCounts);
        telemetry.update();

        // Resetting encoder counts
        resetMotorEncoderCounts();

        // Setting motor to run in runToPosition
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset Yaw
        //imu.resetYaw(); [Aarush]
        //start();
        while (bl.getCurrentPosition() < encoderAbsCounts) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            double correction = PID_Turn(0,myRobotOrientation.thirdAngle,"off");
            double power = PID_FB(encoderAbsCounts,Math.abs(bl.getCurrentPosition()));

            bl.setPower(power-correction);
            fl.setPower(power-correction);
            double bl_fl = power - correction;

            fr.setPower(power+correction);
            br.setPower(power+correction);
            double fr_br = power + correction;

//            telemetry.addData("power", bl_fl);
//            telemetry.addData("power", fr_br);
//            telemetry.addData("correction", correction);
//            telemetry.update();
            idle();
        }
        //stop();
        //getRuntime();
        turnToZeroAngle();

        // apply zero power to avoid continuous power to the wheels
        setMotorToZeroPower();

        // return current encoder count
        currEncoderCount = bl.getCurrentPosition();
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        currZAngle = myRobotOrientation.thirdAngle;
        telemetry.addData("currEncoderCount", currEncoderCount);
        telemetry.addData("currZAngle", currZAngle);
        telemetry.update();
        return (currEncoderCount);
    }

    //move backwards with gyro correction
    public int moveBackwards_wDistance_wGyro(double DistanceAbsIn,double Mpower)
    {

        double currZAngle = 0;
        int currEncoderCount = 0;

        double encoderAbsCounts = ENC2DIST *DistanceAbsIn;
        telemetry.addData("Im here",currZAngle);

        // Resetting encoder counts
        resetMotorEncoderCounts();

        // Setting motor to run in runToPosition
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "RUN_WITHOUT_ENCODER");
        telemetry.update();

        // Reset Yaw
        //imu.resetYaw(); // [Aarush]

        //start();
        while(bl.getCurrentPosition() > -encoderAbsCounts) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            double correction = PID_Turn(0,myRobotOrientation.thirdAngle,"off");
            double power = PID_FB(encoderAbsCounts,Math.abs(bl.getCurrentPosition()));
            bl.setPower(-power-correction);
            fl.setPower(-power-correction);
            double bl_fl = power - correction;

            fr.setPower(-power+correction);
            br.setPower(-power+correction);
            double fr_br = power + correction;

            telemetry.addData("Left_Power", bl_fl);
            telemetry.addData("Right_Power", fr_br);
            telemetry.addData("correction", correction);
            telemetry.update();
            idle();
        }

        turnToZeroAngle();
        // apply zero power to avoid continuous power to the wheels
        setMotorToZeroPower();

        // return current encoder count
        currEncoderCount = bl.getCurrentPosition();
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        currZAngle = myRobotOrientation.thirdAngle;
        telemetry.addData("currEncoderCount", currEncoderCount);
        telemetry.addData("currZAngle", currZAngle);
        telemetry.update();
        return (currEncoderCount);
    }

    public void clawClosed()
    {
        s1.setPosition(0.2);
    }


    public void wristFlat()
    {
        s2.setPosition(0.147);
    }

    public void clawOpen()
    {
        s1.setPosition(0.4);
    }

    public void wristBent()
    {
        s2.setPosition(0.427);
    }

    public void turn(String direction, double targetAngle)
    {
        imu.resetYaw();
        if (direction.equalsIgnoreCase("right")){
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            telemetry.addData("Turn", "Right");
            telemetry.addData("Initial Angle",myRobotOrientation.thirdAngle);
            telemetry.update();
            while (Math.abs(myRobotOrientation.thirdAngle) <= targetAngle) {
                myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                double power = PID_Turn(targetAngle, Math.abs(myRobotOrientation.thirdAngle),"on");
                bl.setPower(power);
                fl.setPower(power);
                fr.setPower(-power);
                br.setPower(-power);
            }
            telemetry.addData("Turn", "Right");
            telemetry.addData("Current Angle",myRobotOrientation.thirdAngle);
            telemetry.update();
            bl.setPower(0);
            fl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        } else if(direction.equalsIgnoreCase("left")){
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            telemetry.addData("Turn", "Right");
            telemetry.addData("Initial Angle",myRobotOrientation.thirdAngle);
            telemetry.update();

            while (Math.abs(myRobotOrientation.thirdAngle) <= targetAngle) {
                myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                double power = PID_Turn(targetAngle, Math.abs(myRobotOrientation.thirdAngle),"on");
                bl.setPower(-power);
                fl.setPower(-power);
                fr.setPower(power);
                br.setPower(power);
            }
            telemetry.addData("Turn", "Left");
            telemetry.addData("Current Angle",myRobotOrientation.thirdAngle);
            telemetry.update();
            bl.setPower(0);
            fl.setPower(0);
            fr.setPower(0);
            br.setPower(0);

        }
        imu.resetYaw(); // [ AARUSH ]

    }

    public void turnToZeroAngle()
    {
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double targetAngle = myRobotOrientation.thirdAngle;
        if (targetAngle > 0)
        {
            telemetry.addData("targetAnge",targetAngle);
            telemetry.update();
            turn("right", Math.abs(targetAngle));
        }
        else if (targetAngle < 0)
        {
            telemetry.addData("targetAnge",targetAngle);
            telemetry.update();
            turn("left", Math.abs(targetAngle));
        }
        imu.resetYaw();
    }
    public void intake(int t_msec )
    {
        m0.setDirection(DcMotor.Direction.FORWARD);
        m1.setDirection(DcMotor.Direction.REVERSE);
        m0.setPower(1);
        m1.setPower(1);
        sleep(t_msec);
        m0.setPower(0);
        m1.setPower(0);

    }


    public int moveSideways_wCorrection(String direction, int DistanceAbsIn, double motorAbsPower)
    {
        turnToZeroAngle();
        int currEncoderCount = 0;
        double encoderAbsCounts = ENC2DIST_SIDEWAYS*DistanceAbsIn; //2000/42
        setMotorOrientation();
        // Resetting encoder counts
        resetMotorEncoderCounts();
        telemetry.addData("Encoder count target",encoderAbsCounts);

        // Setting motor to run in runToPosition\
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "RUN_WITHOUT_ENCODER");
        telemetry.update();



        // Wait for robot to finish this movement
        telemetry.addData("encoderAbsCounts (target)", encoderAbsCounts);
        telemetry.addData("currEncoderCount (initial)",bl.getCurrentPosition());



        telemetry.update();
        double refEC = 0;
        while (refEC < encoderAbsCounts) {

            double frEC = fr.getCurrentPosition();
            double blEC = bl.getCurrentPosition();
            double flEC = fl.getCurrentPosition();
            double brEC = br.getCurrentPosition();
            double frCorr = 1;
            double blCorr = 1;
            double flCorr = 1;
            double brCorr = 1;
            if (frEC != 0 ) {
                frEC = Math.abs(frEC);
                refEC = frEC;
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
            double flPow = -motorAbsPower * flCorr;
            double blPow = motorAbsPower * blCorr;
            double frPow = motorAbsPower * frCorr;
            double brPow = -motorAbsPower * brCorr;

            if (direction.equalsIgnoreCase("left")) {
                fl.setPower(flPow);
                bl.setPower(blPow);
                fr.setPower(frPow);
                br.setPower(brPow);
            }
            else if (direction.equalsIgnoreCase("right")) {
                fl.setPower(-flPow);
                bl.setPower(-blPow);
                fr.setPower(-frPow);
                br.setPower(-brPow);
            }
            idle();
        }
        telemetry.addData("enc-fl",fl.getCurrentPosition()); // we dont get encoder counts in fl
        telemetry.addData("enc-br",br.getCurrentPosition()); // possibly, running faster
        telemetry.addData("enc-bl",bl.getCurrentPosition());
        telemetry.addData("enc-fr",fr.getCurrentPosition());
        telemetry.update();
        turnToZeroAngle();

        // apply zero power to avoid continuous power to the wheels
        setMotorToZeroPower();

        // return current encoder count
        currEncoderCount = bl.getCurrentPosition();
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        telemetry.addData("currEncoderCount (final)", currEncoderCount);

        telemetry.update();
        return (currEncoderCount);
    }

    public void encoder_test(double encoderAbsCounts)
    {
        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        resetMotorEncoderCounts();
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (bl.getCurrentPosition() > -encoderAbsCounts) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            bl.setPower(-0.3);
            fl.setPower(-0.3);
            fr.setPower(-0.3);
            br.setPower(-0.3);
            telemetry.update();
            idle();
        }
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);


        telemetry.addData("encoder count b1",bl.getCurrentPosition());
        telemetry.addData("encoder count br",br.getCurrentPosition());
        telemetry.addData("encoder count fl",fl.getCurrentPosition());
        telemetry.addData("encoder count fr",fr.getCurrentPosition());
        telemetry.update();

    }

    public void extend(double power, int encoderAbsCounts) {
        m2.setDirection(DcMotor.Direction.FORWARD);
        m3.setDirection(DcMotor.Direction.FORWARD);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Start count", m2.getCurrentPosition());
        telemetry.addData("Start count", m3.getCurrentPosition());
        telemetry.update();

        while (m2.getCurrentPosition() > -encoderAbsCounts){
            m2.setPower(-power);
            m3.setPower(power);
            telemetry.addData("Count M2",m2.getCurrentPosition());
            telemetry.addData("Count M3",m3.getCurrentPosition());
            telemetry.update();
            idle();
        }
        m2.setPower(0); // set power to 0 so the motor stops running
        m3.setPower(0);

    }

    public void retract(double power, int encoderAbsCounts) {
        m2.setDirection(DcMotor.Direction.FORWARD);
        m3.setDirection(DcMotor.Direction.FORWARD);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Start count", m2.getCurrentPosition());
        telemetry.addData("Start count", m3.getCurrentPosition());
        telemetry.update();

        while (m2.getCurrentPosition() < encoderAbsCounts){
            m2.setPower(power);
            m3.setPower(-power);
            telemetry.addData("Count M2",m2.getCurrentPosition());
            telemetry.addData("Count M3",m3.getCurrentPosition());
            telemetry.update();
            idle();
        }
        m2.setPower(0); // set power to 0 so the motor stops running
        m3.setPower(0);

    }

    public void realign_Sideways(String direction){


        if (direction.equalsIgnoreCase("left")) {
            bl.setPower(-0.2);
            fl.setPower(0.2);
            fr.setPower(0.2);
            br.setPower(-0.2);
        }
        else if (direction.equalsIgnoreCase("right")) {
            bl.setPower(0.2);
            fl.setPower(-0.2);
            fr.setPower(-0.2);
            br.setPower(0.2);
        }
        sleep(1500);
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

    }

    public void realign_FB(String direction){
        if (direction.equalsIgnoreCase("forward")) {
            bl.setPower(0.2);
            fl.setPower(0.2);
            fr.setPower(0.2);
            br.setPower(0.2);
        }
        else if (direction.equalsIgnoreCase("backward")) {
            bl.setPower(-0.2);
            fl.setPower(-0.2);
            fr.setPower(-0.2);
            br.setPower(-0.2);
        }
        sleep(1500);
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

    }


    @Override
    public void runOpMode() throws InterruptedException {
    }


}
