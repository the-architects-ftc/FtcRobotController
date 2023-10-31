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
import com.qualcomm.robotcore.util.ElapsedTime;

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


@Autonomous(name="GyroWorkingSample", group="Linear Opmode2")
public class GyroWorkingSample extends LinearOpMode {

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

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(false);
        // Variable declaration
        BHI260IMU.Parameters myIMUParameters;

        // Start imu initialization
        telemetry.addData("Status", "Start initialization");
        telemetry.update();
        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,RevHubOrientationOnRobot.UsbFacingDirection.FORWARD )
        );

        imu = hardwareMap.get(BHI260IMU.class,"imu");
        bl = hardwareMap.get(DcMotor.class, "LB");
        fl = hardwareMap.get(DcMotor.class, "LF");
        fr = hardwareMap.get(DcMotor.class, "RF");
        br = hardwareMap.get(DcMotor.class, "RB");
        bl.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        imu.initialize(myIMUParameters);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double LF = 0;
            double RF = 0;
            double LB = 0;
            double RB = 0;
            double initialAngle = 0;

            // Now use these simple methods to extract each angle
            // (Java type double) from the object you just created:
            robotOrientation = imu.getRobotYawPitchRollAngles();
            double Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double Roll = robotOrientation.getRoll(AngleUnit.DEGREES);
            telemetry.addData("Yaw", Yaw);
            telemetry.addData("Pitch", Pitch);
            telemetry.addData("Roll", Roll);
            telemetry.update();


            imu.resetYaw();

            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            float X_axis = myRobotOrientation.firstAngle;
            float Y_axis = myRobotOrientation.secondAngle;
            float Z_axis = myRobotOrientation.thirdAngle;
            //telemetry.addData("X_axis",X_axis);
            //telemetry.addData("Y_axis",Y_axis);
            telemetry.addData("Z_axis", Z_axis);
            telemetry.update();

            telemetry.addData("Moving the robot while in sleep", 0);
            telemetry.update();


//            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES);
//            X_axis = myRobotOrientation.firstAngle;
//            Y_axis = myRobotOrientation.secondAngle;
//            Z_axis = myRobotOrientation.thirdAngle;
//            //telemetry.addData("X_axis",X_axis);
//            //telemetry.addData("Y_axis",Y_axis);
//            telemetry.addData("Z_axis",Z_axis);
//            telemetry.update();
//
//            sleep(10000);
//
//            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES);
//            X_axis = myRobotOrientation.firstAngle;
//            Y_axis = myRobotOrientation.secondAngle;
//            Z_axis = myRobotOrientation.thirdAngle;
//            //telemetry.addData("X_axis",X_axis);
//            //telemetry.addData("Y_axis",Y_axis);
//            telemetry.addData("Z_axis",Z_axis);
//            telemetry.update();
//            sleep(10000);
//            initialAngle = myRobotOrientation.thirdAngle;
//            telemetry.addData("Initial Angle", initialAngle);
//
//
//                while (myRobotOrientation.thirdAngle <= 90) {
//                    myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//                    //telemetry.addData("Current Angle",myRobotOrientation.thirdAngle);
//                    //telemetry.update();
//                    double power = calculatePower(90, myRobotOrientation.thirdAngle);
//                    bl.setPower(power);
//                    fl.setPower(power);
//                    fr.setPower(-power);
//                    br.setPower(-power);
//
//                }
//
//
//            bl.setPower(0);
//            fl.setPower(0);
//            fr.setPower(0);
//            br.setPower(0);

//            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES);
//            telemetry.addData("Last Angle",myRobotOrientation.thirdAngle);
//            telemetry.update();

            turn("right",90,imu);
            sleep(5000);
            turn("left",90,imu);
            sleep(100000000);


        }


    }

    public double calculatePower(double targetAngle, double currentAngle){
        double power = 0.7*(1-(currentAngle/targetAngle));
        if (power < 0.2){
            power = 0.2;
        }
        telemetry.addData("Calculated Power",power);
        telemetry.update();
        return power;

    }

    public void turn(String direction, double targetAngle, BHI260IMU imu){
        imu.resetYaw();
        if (direction.equalsIgnoreCase("left")){
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            while (myRobotOrientation.thirdAngle <= targetAngle) {
                myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                //telemetry.addData("Current Angle",myRobotOrientation.thirdAngle);
                //telemetry.update();
                double power = calculatePower(targetAngle, myRobotOrientation.thirdAngle);
                bl.setPower(power);
                fl.setPower(power);
                fr.setPower(-power);
                br.setPower(-power);



            }
            bl.setPower(0);
            fl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        } else if(direction.equalsIgnoreCase("right")){
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            while (myRobotOrientation.thirdAngle >= -targetAngle) {
                myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                //telemetry.addData("Current Angle",myRobotOrientation.thirdAngle);
                //telemetry.update();
                double power = calculatePower(-targetAngle, myRobotOrientation.thirdAngle);
                bl.setPower(-power);
                fl.setPower(-power);
                fr.setPower(power);
                br.setPower(power);

            }
            bl.setPower(0);
            fl.setPower(0);
            fr.setPower(0);
            br.setPower(0);

        }


    }
}




