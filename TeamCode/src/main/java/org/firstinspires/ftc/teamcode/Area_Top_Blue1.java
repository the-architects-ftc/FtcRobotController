/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//MINE ( AARUSH )
package org.firstinspires.ftc.teamcode;


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


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.

 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.

 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Autonomous(name="Area_Top_Blue1", group="Linear Opmode2")
public class Area_Top_Blue1 extends LinearOpMode {

    CommonUtil util = new CommonUtil();
    ElapsedTime runtime = new ElapsedTime();
    BHI260IMU imu;
    //IMU.Parameters myIMUParameters;

    // Create an object to receive the IMU angles
    YawPitchRollAngles robotOrientation;
    Orientation myRobotOrientation;

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

    double ENC2DIST = 2000/48;

    @Override
    public void runOpMode() {

        // Variable declaration
        BHI260IMU.Parameters myIMUParameters;

        //setup
        telemetry.setAutoClear(false);

        // map imu
        imu = hardwareMap.get(BHI260IMU.class,"imu");
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

        // Initialize motors
        setMotorOrientation();
        //resetMotorEncoderCounts();
        s2.setPosition(0);
        s1.setPosition(0.4);
        // Start imu initialization
        telemetry.addData("Gyro Status", "Start initialization");
        telemetry.update();
        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,RevHubOrientationOnRobot.UsbFacingDirection.UP )
        );
        imu.initialize(myIMUParameters);
        telemetry.addData("Gyro Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            moveForward_wDistance_wGyro(15,0.7,ENC2DIST,imu);
            sleep(1000);

            moveBackwards_wDistance_wGyro(15,0.7,ENC2DIST,imu);
            sleep(1000);

            moveSideways_wCorrection("left",15,0.5);
            sleep(1000);

            moveSideways_wCorrection("right",15,0.5);
            sleep(1000);

            turn("right",90,imu);
            sleep(1000);

            turn("left",90,imu);
            sleep(500000);
        }
    }

    // Set motor directions
    private void setMotorOrientation()
    {
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

    }

    // Reset motor encoder counts
    private void resetMotorEncoderCounts()
    {
        // Reset encoder counts kept by motors
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        telemetry.addData("Encoder", "Count Reset");  // telemetry: Mode Waiting
        telemetry.update();
    }

    // Set motor to zero power
    private void setMotorToZeroPower()
    {
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }


    //Uses gyro to self correct while moving forwards
    private int moveForward_wDistance_wGyro(int DistanceAbsIn, double motorAbsPower, double CountToDist, BHI260IMU imu)
    {

        double currZAngle = 0;
        int currEncoderCount = 0;
        double encoderAbsCounts = CountToDist*DistanceAbsIn;
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
        imu.resetYaw();

        // Wait for robot to finish this movement
        telemetry.addData("encoderAbsCounts", encoderAbsCounts);
        telemetry.update();
        telemetry.addData("enc-bl",bl.getCurrentPosition());
        telemetry.update();

        while (bl.getCurrentPosition() < encoderAbsCounts) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            double correction = myRobotOrientation.thirdAngle/180;

            double power = calculatePowerFB(encoderAbsCounts,Math.abs(bl.getCurrentPosition()),motorAbsPower);
            bl.setPower(power-correction);
            fl.setPower(power-correction);
            fr.setPower(power+correction);
            br.setPower(power+correction);
            //telemetry.addData("correction", correction);
            //telemetry.update();
            idle();
        }

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


    //Uses gyro to self correct while moving backwards
    private int moveBackwards_wDistance_wGyro(int DistanceAbsIn, double motorAbsPower,double CountToDist, BHI260IMU imu)
    {

        double currZAngle = 0;
        int currEncoderCount = 0;

        double encoderAbsCounts = CountToDist*DistanceAbsIn;
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
        imu.resetYaw();

        // Wait for robot to finish this movement
        telemetry.addData("encoderAbsCounts", encoderAbsCounts);
        telemetry.update();
        telemetry.addData("enc-bl",bl.getCurrentPosition());
        telemetry.update();

        while(bl.getCurrentPosition() > -encoderAbsCounts) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            double correction = myRobotOrientation.thirdAngle/180;
            double power = calculatePowerFB(encoderAbsCounts,Math.abs(bl.getCurrentPosition()),motorAbsPower);
            bl.setPower(-power-correction);
            fl.setPower(-power-correction);
            fr.setPower(-power+correction);
            br.setPower(-power+correction);
            //telemetry.addData("correction", correction);
            //telemetry.update();
            idle();
        }

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

    //self explained
    private void stayPut(int t_msec)
    {
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        sleep(t_msec);

    }

    //Move right using time measurement ( not accurate )
    private void moveRight_dist(double d_inch)
    {
        long t_msec = 0;
        double temp = 0;
        // 500 msec moves the robot 24 inches
        temp = (500.0/13.5)*d_inch;
        t_msec = (long) temp;
        // Set powers
        bl.setPower(0.5);
        fl.setPower(-0.5);
        fr.setPower(0.5);
        br.setPower(-0.5);
        sleep(t_msec);
        stayPut(500);
    }

    //Move left using time measurement ( not accurate )
    private void moveLeft_dist(double d_inch)
    {
        long t_msec = 0;
        double temp = 0;
        // 500 msec moves the robot 24 inches
        temp = (500.0/13.5)*d_inch;
        t_msec = (long) temp;
        // Set powers
        bl.setPower(-0.5);
        fl.setPower(0.5);
        fr.setPower(-0.5);
        br.setPower(0.5);
        sleep(t_msec);
        stayPut(500);
    }


    //Calculate power for gyro
    public double calculatePower(double targetAngle, double currentAngle)
    {
        double power = 0.7*(1-(currentAngle/targetAngle));
        if (power < 0.2){
            power = 0.2;
        }
        telemetry.addData("Calculated Power",power);
        telemetry.update();
        return power;

    }
    public double calculatePowerFB(double targetEC, double currentEC,double motorMaxPower)
    {
        double power = motorMaxPower*(1-(targetEC/currentEC));
        if (power < 0.4){
            power = 0.4;
        }
        telemetry.addData("Calculated Power",power);
        telemetry.update();
        return power;

    }


    //TURN WITH GYRO
    public void turn(String direction, double targetAngle, BHI260IMU imu)
    {
        imu.resetYaw();
        if (direction.equalsIgnoreCase("right")){
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            telemetry.addData("Turn", "Right");
            telemetry.addData("Initial Angle",myRobotOrientation.thirdAngle);
            telemetry.update();
            while (Math.abs(myRobotOrientation.thirdAngle) <= targetAngle) {
                myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                double power = calculatePower(targetAngle, Math.abs(myRobotOrientation.thirdAngle));
                bl.setPower(power);
                fl.setPower(power);
                fr.setPower(-power);
                br.setPower(-power);
                telemetry.addData("Turn", "Right");
                telemetry.addData("Current Angle",myRobotOrientation.thirdAngle);
                telemetry.addData("power", power);
                telemetry.update();
            }
            bl.setPower(0);
            fl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        } else if(direction.equalsIgnoreCase("left")){
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            while (Math.abs(myRobotOrientation.thirdAngle) <= targetAngle) {
                myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                telemetry.addData("Current Angle",myRobotOrientation.thirdAngle);
                telemetry.update();
                double power = calculatePower(targetAngle, Math.abs(myRobotOrientation.thirdAngle));
                bl.setPower(-power);
                fl.setPower(-power);
                fr.setPower(power);
                br.setPower(power);
                telemetry.addData("Turn", "Left");
                telemetry.addData("Current Angle",myRobotOrientation.thirdAngle);
                telemetry.addData("power", power);
                telemetry.update();

            }
            bl.setPower(0);
            fl.setPower(0);
            fr.setPower(0);
            br.setPower(0);

        }
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

    private int moveSideways_wCorrection(String direction, int DistanceAbsIn, double motorAbsPower)
    {

        int currEncoderCount = 0;
        double encoderAbsCounts = 2000/42*DistanceAbsIn;
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

        // Reset Yaw

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

        // apply zero power to avoid continuous power to the wheels
        setMotorToZeroPower();

        // return current encoder count
        currEncoderCount = bl.getCurrentPosition();
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        telemetry.addData("currEncoderCount (final)", currEncoderCount);

        telemetry.update();
        return (currEncoderCount);
    }



    private int moveLeft_wGyro(int DistanceAbsIn, double motorAbsPower,BHI260IMU imu)
    {
        double currZAngle = 0;
        int currEncoderCount = 0;
        double encoderAbsCounts = (DistanceAbsIn/10.0)*500.0;
        // Resetting encoder counts
        resetMotorEncoderCounts();
        telemetry.addData("Target encoder counts",encoderAbsCounts);

        // Setting motor to run in runToPosition\
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "RUN_WITHOUT_ENCODER");
        telemetry.update();

        // Reset Yaw
        imu.resetYaw();

        // Wait for robot to finish this movement
        telemetry.addData("encoderAbsCounts", encoderAbsCounts);
        telemetry.update();
        telemetry.addData("enc-bl",bl.getCurrentPosition());
        telemetry.update();
        while (bl.getCurrentPosition() > -encoderAbsCounts) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            double correction = myRobotOrientation.thirdAngle/180;
            correction = 0;
            bl.setPower(-motorAbsPower-correction);
            fl.setPower(motorAbsPower-correction);
            fr.setPower(-motorAbsPower+correction);
            br.setPower(motorAbsPower+correction);
            telemetry.addData("correction", correction);
            telemetry.update();
            idle();
        }
        telemetry.addData("bl power:", bl.getPower());
        telemetry.addData("fl power:", fl.getPower());
        telemetry.addData("fr power:", fr.getPower());
        telemetry.addData("br power:", br.getPower());
        telemetry.update();

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

    private void encoder_test(double encoderAbsCounts)
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

    private void extend(double power, int encoderAbsCounts) {
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




}




