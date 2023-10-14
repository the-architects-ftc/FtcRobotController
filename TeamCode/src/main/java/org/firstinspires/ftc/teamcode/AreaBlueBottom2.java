//Bottom Blue section --> Middle line
// Robot starts facing the white pixels
// Its moves to the side to place the purple pixel and then gets a white pixel
// After getting the white pixel the robot goes through the stage door and goes to the backdrop and places the pixel
//Finally, it parks in the parking area
//MINE ( AARUSH )
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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


@Autonomous(name="AreaBlueBottom2", group="Linear Opmode1")
public class AreaBlueBottom2 extends LinearOpMode {


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


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);


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


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.


            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            // LF    = Range.clip(drive + turn, -1.0, 1.0) ;
            // RF   = Range.clip(drive - turn, -1.0, 1.0) ;

// Moving left:
            if (runtime.seconds() > 0) {
                LF = -0.5;
                RF = 0.5;
                LB = 0.5;
                RB = -0.5;
            }



            //Stopping ( placing )
            if (runtime.seconds() > 1) {
                LF = 0;
                RF = 0;
                LB = 0;
                RB = 0;
            }

            //Forwards
            if (runtime.seconds() > 2) {
                LF = 0.5;
                RF = 0.5;
                LB = 0.5;
                RB = 0.5;
            }

            //stop
            if (runtime.seconds() > 3) {
                LF = 0;
                RF = 0;
                LB = 0;
                RB = 0;
            }


            // Moving left:
            if (runtime.seconds() > 4) {
                LF = -0.5;
                RF = 0.5;
                LB = 0.5;
                RB = -0.5;
            }

            //Backwards
            if (runtime.seconds() > 5) {
                LF = -0.5;
                RF = -0.5;
                LB = -0.5;
                RB = -0.5;
            }

            //stopping
            if (runtime.seconds() > 6) {
                LF = 0;
                RF = 0;
                LB = 0;
                RB = 0;
            }
// Moving RIGHT:
            if (runtime.seconds() > 7) {
                LF = 0.5;
                RF = -0.5;
                LB = -0.5;
                RB = 0.5;
            }

            //stopping
            if (runtime.seconds() > 8) {
                LF = 0;
                RF = 0;
                LB = 0;
                RB = 0;
            }
//BACK
            if (runtime.seconds() > 9) {
                LF = -0.5;
                RF = -0.5;
                LB = -0.5;
                RB = -0.5;
            }
//STOP
            if (runtime.seconds() > 10) {
                LF = 0;
                RF = 0;
                LB = 0;
                RB = 0;
            }







            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // LF  = -gamepad1.left_stick_y ;
            // RF = -gamepad1.right_stick_y ;


            // Send calculated power to wheels
            bl.setPower(LB);
            fl.setPower(LF);
            fr.setPower(RF);
            br.setPower(RB);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", LF, RF,LB,RB);
            telemetry.update();
        }
    }
}

