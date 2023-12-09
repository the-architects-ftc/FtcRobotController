@ -125,7 +125,7 @@ public class Arm_test extends LinearOpMode {
        while (opModeIsActive()) {
//CODE STARTS HERE :)

        extend(1);
        extend(0.1,3);
        sleep(10000);
    }
}
@ -142,17 +142,27 @@ public class Arm_test extends LinearOpMode {
        br.setDirection(DcMotor.Direction.REVERSE);
}

    private void extend (double power)
    private void extend (double power, int encoderAbsCounts)
    {
        int currEncoderCount = 0;

        m2.setDirection(DcMotor.Direction.FORWARD);
        m3.setDirection(DcMotor.Direction.FORWARD);
        m3.setPower(power);
        m2.setPower(power);
        sleep(3050);
        m3.setPower(0);
        m2.setPower(0);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (m2.getCurrentPosition() < encoderAbsCounts) {
            m2.setPower(power);
            idle();
        }

        telemetry.addData("Desired count",encoderAbsCounts);
        telemetry.update();
        telemetry.addData("Current count",m2.getCurrentPosition());
        telemetry.update();

        m2.setPower(0);

        sleep(3050);
    }

    private void revert (double power)
@ -180,6 +190,7 @@ public class Arm_test extends LinearOpMode {
        fl.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        telemetry.addData("Encoder", "Count Reset");  // telemetry: Mode Waiting
        telemetry.update();
}
