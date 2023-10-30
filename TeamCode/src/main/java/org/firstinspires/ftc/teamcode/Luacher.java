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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


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


@Autonomous(name="Luacher", group="Linear Opmode2")
public class Luacher extends LinearOpMode {


    // Declare OpMode members.
    //private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor bl = null;
    //private DcMotor fl = null;
    //private DcMotor fr = null;
    //private DcMotor br = null;

    ElapsedTime runtime = new ElapsedTime();
    DcMotor bl = null;
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor br = null;
    DcMotor m0 = null;
    DcMotor m1 = null;




    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        bl = hardwareMap.get(DcMotor.class, "LB");
        fl = hardwareMap.get(DcMotor.class, "LF");
        fr = hardwareMap.get(DcMotor.class, "RF");
        br = hardwareMap.get(DcMotor.class, "RB");
        m0 = hardwareMap.get(DcMotor.class, "M0");
        m1 = hardwareMap.get(DcMotor.class, "M1");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        m0.setDirection(DcMotor.Direction.FORWARD);
        m1.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Setup a variable for each drive wheel to save power level for telemetry
            double LF = 0;
            double RF = 0;
            double LB = 0;
            double RB = 0;
            double M0 = 0;
            double M1 = 0;
            double currTime = 0;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.


            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            // LF    = Range.clip(drive + turn, -1.0, 1.0) ;
            // RF   = Range.clip(drive - turn, -1.0, 1.0) ;



            //GUYS CODE GHOES HERE JUST SAYING THO
            //LISTEN :)

            launcher_time(5000000);





            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // LF  = -gamepad1.left_stick_y ;
            // RF = -gamepad1.right_stick_y ;


            // Send calculated power to wheels
            //bl.setPower(LB);
            //fl.setPower(LF);
            //fr.setPower(RF);
            //br.setPower(RB);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", LF, RF,LB,RB,M0,M1);
            telemetry.update();
        }
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
        bl.setPower(-0);
        fl.setPower(-0);
        fr.setPower(0);
        br.setPower(0);
        sleep(t_msec);

    }

    //FORWARDS
    private void moveForward_time (int t_msec)
    {
        bl.setPower(5);
        fl.setPower(5);
        fr.setPower(5);
        br.setPower(5);
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
        bl.setPower(-5);
        fl.setPower(-5);
        fr.setPower(-5);
        br.setPower(-5);
        sleep(t_msec);

    }

    private void intake_time (int t_msec)
    {
        bl.setPower(-0.2);
        fl.setPower(-0.2);
        fr.setPower(-0.2);
        br.setPower(-0.2);
        m0.setPower(1);
        m1.setPower(1);
        sleep(t_msec);
    }

    private void launcher_time (int t_msec)
    {
        bl.setPower(0.2);
        fl.setPower(0.2);
        fr.setPower(0.2);
        br.setPower(0.2);
        m0.setPower(-1);
        m1.setPower(-1);
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




