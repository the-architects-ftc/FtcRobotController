//MINE ( AARUSH )
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


//MINE ( AARUSH )


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


@Autonomous(name="Auto_TopRedPI", group="Linear Opmode2")
public class Auto_TopRedPI extends CommonUtil {

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
        setMotorToZeroPower();
        clawOpen();
        wristFlat();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            clawClosed();
            sleep(500);
            extend(1,150);
            clawClosed();
            moveSideways_wCorrection("right",1,0.35);
            sleep(500);
            moveBackwards_wDistance_wGyro(30,1);
            sleep(500);
            moveSideways_wCorrection("right",22,0.4);
            sleep(500);
            moveBackwards_wDistance_wGyro(5,1);
            realign_FB("backward");
            extend(1,4500);
            clawClosed();
            wristBent();
            sleep(500);
            clawOpen();
            sleep(1200);
            clawClosed();
            sleep(500);
            wristFlat();
            retract(1,4000);
            sleep(500);
            wristFlat();
            moveForward_wDistance_wGyro(5,1);
            sleep(500);
            moveSideways_wCorrection("right",22,0.6);
            sleep(500000);
            moveBackwards_wDistance_wGyro(10,0.5);
            sleep(500000);

        }
    }


}





