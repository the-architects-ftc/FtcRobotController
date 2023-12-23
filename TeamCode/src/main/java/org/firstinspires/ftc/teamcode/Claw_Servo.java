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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="Claw_Servo", group="Linear Opmode2")
public class Claw_Servo extends LinearOpMode
{
    DcMotor bl = null;
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor br = null;
    //Servo s1 = null;
    Servo s1 = null;



    @Override
    public void runOpMode()
    {

        s1 = hardwareMap.get(Servo.class, "s1");
        s1.setDirection(Servo.Direction.FORWARD);

        s1.setPosition(0.);
        sleep(30000);

    }


    // Set motor directions
    private void setMotorOrientation()
    {
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        bl.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
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

    private void reset_servo()
    {
        //s1.setPosition(0);
    }

    private void move_Servo ( double Angle)
    {
        double position = Angle/180;
        reset_servo();
        //s1.setPosition(position);

        //telemetry.addData("Servo position:", s1.getPosition());
    }

    // Turn 90 degree Left
    private void turn90Left()
    {
        bl.setPower(0.5);
        fl.setPower(0.5);
        fr.setPower(-0.5);
        br.setPower(-0.5);
        sleep(700);

    }

    // Turn 90 degree Right
    private void turn90Right()
    {
        bl.setPower(-0.5);
        fl.setPower(-0.5);
        fr.setPower(0.5);
        br.setPower(0.5);
        sleep(700);

    }


    // Move forward by encoder counts
    private int moveForward_wEncoder_counts(int encoderAbsCounts, double motorAbsPower)
    {
        int currEncoderCount = 0;
        // Resetting encoder counts
        resetMotorEncoderCounts();

        // Setting motor to run in runToPosition
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "RUN_WITHOUT_ENCODER");
        telemetry.update();

        // Set motor power
        // NOTE: sign of power is ignored as sign of target encoder position controls direction when running to position
        bl.setPower(motorAbsPower);
        fl.setPower(motorAbsPower);
        fr.setPower(motorAbsPower);
        br.setPower(motorAbsPower);

        // Wait for robot to finish this movement
        while (opModeIsActive() && (fl.getCurrentPosition() < encoderAbsCounts)) {
            telemetry.addData("bl power:", bl.getPower());
            telemetry.addData("fl power:", fl.getPower());
            telemetry.addData("fr power:", fr.getPower());
            telemetry.addData("br power:", br.getPower());
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
        currEncoderCount = fl.getCurrentPosition();
        telemetry.addData("currEncoderCount", currEncoderCount);
        telemetry.update();
        return (currEncoderCount);
    }

    private int moveForward_wDistance(int DistanceAbsIn, double motorAbsPower)
    {
        int currEncoderCount = 0;
        double encoderAbsCounts = (DistanceAbsIn/12.5)*500.0;
        // Resetting encoder counts
        resetMotorEncoderCounts();

        // Setting motor to run in runToPosition\
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "RUN_WITHOUT_ENCODER");
        telemetry.update();

        // Set motor power
        // NOTE: sign of power is ignored as sign of target encoder position controls direction when running to position
        bl.setPower(motorAbsPower);
        fl.setPower(motorAbsPower);
        fr.setPower(motorAbsPower);
        br.setPower(motorAbsPower);

        // Wait for robot to finish this movement
        telemetry.addData("encoderAbsCounts", encoderAbsCounts);
        telemetry.update();
        telemetry.addData("enc-bl",bl.getCurrentPosition());
        telemetry.update();
        while (opModeIsActive() && (bl.getCurrentPosition() < encoderAbsCounts)) {
            telemetry.addData("enc-bl",bl.getCurrentPosition());
            telemetry.addData("enc-fl",fl.getCurrentPosition());
            telemetry.addData("enc-fr",fr.getCurrentPosition());
            telemetry.addData("enc-br",br.getCurrentPosition());
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
        telemetry.addData("currEncoderCount", currEncoderCount);
        telemetry.update();
        return (currEncoderCount);
    }
    // Moving Backwards
    private int moveBackward_wEncoder_counts(int encoderAbsCounts, double motorAbsPower)
    {
        int currEncoderCount = 0;

        // Resetting encoder counts
        resetMotorEncoderCounts();

        // Setting motor to run in runToPosition
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status","RUN_WITHOUT_ENCODER");
        telemetry.update();

        // Set motor power
        // NOTE: sign of power is ignored as sign of target encoder position controls direction when running to position
        bl.setPower(-motorAbsPower);
        fl.setPower(-motorAbsPower);
        fr.setPower(-motorAbsPower);
        br.setPower(-motorAbsPower);

        // Wait for robot to finish this movement
        while (opModeIsActive() && (fl.getCurrentPosition()>-encoderAbsCounts))
        {
            //telemetry.addData("enc-bl",fl.getCurrentPosition());
            //telemetry.update();
            idle();
        }

        telemetry.addData("bl power:",bl.getPower());
        telemetry.addData("fl power:",fl.getPower());
        telemetry.addData("fr power:",fr.getPower());
        telemetry.addData("br power:",br.getPower());
        telemetry.update();

        // apply zero power to avoid continuous power to the wheels
        setMotorToZeroPower();

        // return current encoder count
        currEncoderCount = fl.getCurrentPosition();
        telemetry.addData("currEncoderCount",currEncoderCount);
        telemetry.update();
        return(currEncoderCount);

    }


    private int moveBackward_wDistance(int DistanceAbsIn, double motorAbsPower)
    {
        int currEncoderCount = 0;
        double encoderAbsCounts = (DistanceAbsIn/12.5)*500.1;

        // Resetting encoder counts
        resetMotorEncoderCounts();

        // Setting motor to run in runToPosition
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status","RUN_WITHOUT_ENCODER");
        telemetry.update();

        // Set motor power
        // NOTE: sign of power is ignored as sign of target encoder position controls direction when running to position
        bl.setPower(-motorAbsPower);
        fl.setPower(-motorAbsPower);
        fr.setPower(-motorAbsPower);
        br.setPower(-motorAbsPower);

        // Wait for robot to finish this movement
        while (opModeIsActive() && (bl.getCurrentPosition()>-encoderAbsCounts))
        {
            //telemetry.addData("enc-bl",fl.getCurrentPosition());
            //telemetry.update();
            idle();
        }

        telemetry.addData("bl power:",bl.getPower());
        telemetry.addData("fl power:",fl.getPower());
        telemetry.addData("fr power:",fr.getPower());
        telemetry.addData("br power:",br.getPower());
        telemetry.update();

        // apply zero power to avoid continuous power to the wheels
        setMotorToZeroPower();

        // return current encoder count
        currEncoderCount = bl.getCurrentPosition();
        telemetry.addData("currEncoderCount",currEncoderCount);
        telemetry.update();
        return(currEncoderCount);

    }
    private int moveLeft_wDistance(int DistanceAbsIn, double motorAbsPower)
    {
        int currEncoderCount = 0;
        double encoderAbsCounts = (DistanceAbsIn/15.5)*500.0;
        // Resetting encoder counts
        resetMotorEncoderCounts();

        // Setting motor to run in runToPosition\
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "RUN_WITHOUT_ENCODER");
        telemetry.update();

        // Set motor power
        // NOTE: sign of power is ignored as sign of target encoder position controls direction when running to position
        bl.setPower(-motorAbsPower);
        fl.setPower(motorAbsPower);
        fr.setPower(-motorAbsPower);
        br.setPower(motorAbsPower);

        // Wait for robot to finish this movement
        telemetry.addData("encoderAbsCounts", encoderAbsCounts);
        telemetry.update();
        telemetry.addData("enc-bl",bl.getCurrentPosition());
        telemetry.update();
        while (opModeIsActive() && (bl.getCurrentPosition() < encoderAbsCounts)) {
            telemetry.addData("enc-bl",bl.getCurrentPosition());
            telemetry.addData("enc-fl",fl.getCurrentPosition());
            telemetry.addData("enc-fr",fr.getCurrentPosition());
            telemetry.addData("enc-br",br.getCurrentPosition());
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
        telemetry.addData("currEncoderCount", currEncoderCount);
        telemetry.update();
        return (currEncoderCount);
    }
    //Moving left
    private int moveLeft_wEncoder_counts(int encoderAbsCounts, double motorAbsPower)
    {
        int currEncoderCount = 0;

        // Resetting encoder counts
        resetMotorEncoderCounts();

        // Setting motor to run in runToPosition
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "RUN_WITHOUT_ENCODER");
        telemetry.update();

        // Set motor power
        // NOTE: sign of power is ignored as sign of target encoder position controls direction when running to position
        bl.setPower(motorAbsPower);
        fl.setPower(-motorAbsPower);
        fr.setPower(motorAbsPower);
        br.setPower(-motorAbsPower);

        // Wait for robot to finish this movement
        while (opModeIsActive() && (fl.getCurrentPosition() > -encoderAbsCounts)) {
            //telemetry.addData("enc-bl",fl.getCurrentPosition());
            //telemetry.update();
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
        currEncoderCount = fl.getCurrentPosition();
        telemetry.addData("currEncoderCount", currEncoderCount);
        telemetry.update();
        return (currEncoderCount);

    }
    private int moveRight_wDistance(int DistanceAbsIn, double motorAbsPower)
    {
        int currEncoderCount = 0;
        double encoderAbsCounts = (DistanceAbsIn/15.5)*500.0;
        // Resetting encoder counts
        resetMotorEncoderCounts();

        // Setting motor to run in runToPosition\
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "RUN_WITHOUT_ENCODER");
        telemetry.update();

        // Set motor power
        // NOTE: sign of power is ignored as sign of target encoder position controls direction when running to position
        bl.setPower(motorAbsPower);
        fl.setPower(-motorAbsPower);
        fr.setPower(motorAbsPower);
        br.setPower(-motorAbsPower);

        // Wait for robot to finish this movement
        telemetry.addData("encoderAbsCounts", encoderAbsCounts);
        telemetry.update();
        telemetry.addData("enc-bl",bl.getCurrentPosition());
        telemetry.update();
        while (opModeIsActive() && (bl.getCurrentPosition() < encoderAbsCounts)) {
            telemetry.addData("enc-bl",bl.getCurrentPosition());
            telemetry.addData("enc-fl",fl.getCurrentPosition());
            telemetry.addData("enc-fr",fr.getCurrentPosition());
            telemetry.addData("enc-br",br.getCurrentPosition());
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
        telemetry.addData("currEncoderCount", currEncoderCount);
        telemetry.update();
        return (currEncoderCount);
    }
    //Moving Right
    private int moveRight_wEncoder_counts(int encoderAbsCounts, double motorAbsPower)
    {
        int currEncoderCount = 0;

        // Resetting encoder counts
        resetMotorEncoderCounts();

        // Setting motor to run in runToPosition
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status","RUN_WITHOUT_ENCODER");
        telemetry.update();

        // Set motor power
        // NOTE: sign of power is ignored as sign of target encoder position controls direction when running to position
        bl.setPower(motorAbsPower);
        fl.setPower(-motorAbsPower);
        fr.setPower(motorAbsPower);
        br.setPower(-motorAbsPower);

        // Wait for robot to finish this movement
        while (opModeIsActive() && (bl.getCurrentPosition()<encoderAbsCounts))
        {
            telemetry.addData("enc-bl",bl.getCurrentPosition());
            telemetry.update();
            idle();
        }

        telemetry.addData("bl power:",bl.getPower());
        telemetry.addData("fl power:",fl.getPower());
        telemetry.addData("fr power:",fr.getPower());
        telemetry.addData("br power:",br.getPower());
        telemetry.update();

        // apply zero power to avoid continuous power to the wheels
        setMotorToZeroPower();

        // return current encoder count
        currEncoderCount = bl.getCurrentPosition();
        telemetry.addData("currEncoderCount",currEncoderCount);
        telemetry.update();
        return(currEncoderCount);

    }

    //STOP ROBOT
    private void stayPut(int t_msec)
    {
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        sleep(t_msec);

    }

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


}




