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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Autonomous(name="Encoder_Test", group="Linear Opmode2")
public class Encoder_Test extends LinearOpMode
{
    DcMotor bl = null;
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor br = null;

    @Override
    public void runOpMode()
    {
        // Variable
        int currEncoderCount= 0;

        // adding telemetry
        telemetry.setAutoClear(false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        bl = hardwareMap.get(DcMotor.class, "LB");
        fl = hardwareMap.get(DcMotor.class, "LF");
        fr = hardwareMap.get(DcMotor.class, "RF");
        br = hardwareMap.get(DcMotor.class, "RB");

        // Set direction of DC Motors
        setMotorOrientation();

        // Reset encoder counts kept by motors
        resetMotorEncoderCounts();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Mode", "waiting");  // telemetry: Mode Waiting
        telemetry.update();

        // reset runtime
        //runtime.reset(); // reset run time

        // run until the end of the match (driver presses STOP)
        currEncoderCount = moveForward_wEncoder_counts( 500, 0.2);
        telemetry.addData("currEncoderCount",currEncoderCount);
        telemetry.addData("fl encoder count",fl.getCurrentPosition());
        telemetry.addData("bl encoder count",bl.getCurrentPosition());
        telemetry.addData("fr encoder count",fr.getCurrentPosition());
        telemetry.addData("br encoder count",br.getCurrentPosition());
        telemetry.update();

        sleep(2000);

        currEncoderCount = moveForward_wEncoder_counts(-500, 0.2);
        telemetry.addData("currEncoderCount",currEncoderCount);
        telemetry.addData("fl encoder count",fl.getCurrentPosition());
        telemetry.addData("bl encoder count",bl.getCurrentPosition());
        telemetry.addData("fr encoder count",fr.getCurrentPosition());
        telemetry.addData("br encoder count",br.getCurrentPosition());

        telemetry.update();

        sleep(7000);
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

    private void setMotorToZeroPower()
    {
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
    // Move forward by encoder counts
    private int moveForward_wEncoder_counts(int encoder_counts, double motor_power)
    {
        int currEncoderCount= 0;
        double moveDirection = 1; // 1 means forward and -1 means backward

        // Set encoder counts
        //bl.setTargetPosition(encoder_counts);
        //fl.setTargetPosition(encoder_counts);
        //fr.setTargetPosition(encoder_counts);
        //br.setTargetPosition(encoder_counts);
        //telemetry.addData("Encoder","Counts Set");
        //telemetry.update();

        // Setting motor to run in runToPosition
        if (false)
        {
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Status","Encoder RunToPosition");
            telemetry.update();
        }
        else
        {
            bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Status","RUN_WITHOUT_ENCODER");
            telemetry.update();
        }

        // Get direction of movement
        if (fl.getCurrentPosition() > encoder_counts)
        {
            moveDirection = -1.0;
        }

        // Set motor power
        // NOTE: sign of power is ignored as sign of target encoder position controls direction when running to position
        bl.setPower( moveDirection*motor_power);
        fl.setPower(moveDirection*motor_power);
        fr.setPower(moveDirection*motor_power);
        br.setPower(moveDirection*motor_power);

        // Wait for robot to finish this movement
        while (opModeIsActive() && ((moveDirection*fl.getCurrentPosition())<(moveDirection*encoder_counts))) // instead of fl.isbusy()
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

    // Turn 90 degree Right
    private void turn90Right()
    {
        bl.setPower(-0.5);
        fl.setPower(-0.5);
        fr.setPower(0.5);
        br.setPower(0.5);
        sleep(700);

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

    //STOP ROBOT
    private void stayPut(int t_msec)
    {
        bl.setPower(0);
        fl.setPower(0);
       fr.setPower(0);
        br.setPower(0);
        sleep(t_msec);

    }

    //FORWARDS
    private void moveForward_time (int t_msec)
    {
        bl.setPower(0.5);
        fl.setPower(0.5);
        fr.setPower(0.5);
        br.setPower(0.5);
        sleep(t_msec);
    }

    private void moveForward_dist(double d_inch)
    {
        long t_msec = 0;
        double temp = 0;
        // 500 msec moves the robot 24 inches
        temp = (500.0/24.0)*d_inch;
        t_msec = (long) temp;
        // Set powers
        bl.setPower(0.5);
        fl.setPower(0.5);
        fr.setPower(0.5);
        br.setPower(0.5);
        sleep(t_msec);
        stayPut(500);
    }

    //BACKWARD
    private void moveBackward_time (int t_msec)
    {
        bl.setPower(-0.5);
        fl.setPower(-0.5);
        fr.setPower(-0.5);
        br.setPower(-0.5);
        sleep(t_msec);

    }


    private void moveBackward_dist(double d_inch)
    {
        long t_msec = 0;
        double temp = 0;
        // 500 msec moves the robot 24 inches
        temp = (500.0/24.0)*d_inch;
        t_msec = (long) temp;
        // Set powers
        bl.setPower(-0.5);
        fl.setPower(-0.5);
        fr.setPower(-0.5);
        br.setPower(-0.5);
        sleep(t_msec);
        stayPut(500);
    }

    private void moveRight_time (int t_msec)
    {
        bl.setPower(-0.5);
        fl.setPower(0.5);
        fr.setPower(-0.5);
        br.setPower(0.5);
        sleep(t_msec);

    }

    private void moveLeft_time (int t_msec)
    {
        bl.setPower(0.5);
        fl.setPower(-0.5);
        fr.setPower(0.5);
        br.setPower(-0.5);
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




