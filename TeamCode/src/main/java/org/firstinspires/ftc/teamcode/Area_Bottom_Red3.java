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


@Autonomous(name="Area_Bottom_Red3", group="Linear Opmode2")
public class Area_Bottom_Red3 extends CommonUtil {

    Orientation myRobotOrientation;

    @Override
    public void runOpMode() {

        //setup
        telemetry.setAutoClear(false);
        // initialize hardware
        initialize(hardwareMap);
        // Initialize motors
        setMotorOrientation();
        //resetMotorEncoderCounts();
        clawOpen();
        wristFlat();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {


            clawClosed();
            sleep(600);

            extend(1,200);
            sleep(200);

            moveBackwards_wDistance_wGyro(17,0.35);
            sleep(500);

            turn("right",90);
            sleep(1000);

            moveSideways_wCorrection("right",5,0.35);
            sleep(500);

            moveBackwards_wDistance_wGyro(1,0.35);
            sleep(200);

            extend(1,300);
            sleep(500);

            retract(1,300);
            sleep(500);

            moveForward_wDistance_wGyro(1,0.35);
            sleep(500);


            moveSideways_wCorrection("right",23,0.5);
            sleep(500);

            moveBackwards_wDistance_wGyro(1,0.35);
            sleep(200);


            moveBackwards_wDistance_wGyro(66,0.8);
            sleep(1000);


            moveSideways_wCorrection("left",30,0.4);
            sleep(500);

            moveBackwards_wDistance_wGyro(2,0.2);

            extend(1,4800);
            sleep(1000);

            clawClosed();
            wristBent();
            sleep(1000);

            clawOpen();
            sleep(1000);

            moveForward_wDistance_wGyro(3,0.7);
            sleep(1000);

            clawClosed();
            wristFlat();

            moveSideways_wCorrection("left",20,0.4);
            sleep(500);

            moveBackwards_wDistance_wGyro(10,0.5);
            sleep(500000);

        }
    }


}




