package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;


import android.graphics.Color;


@TeleOp(name = "ColorSensor", group = "Sensor")
public class ColorSensor extends LinearOpMode {


    @Override
    public void runOpMode() {


        // Get the color sensor named "sensor_color" in your configuration
        NormalizedColorSensor colorSensor0 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor0");
        NormalizedColorSensor colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        NormalizedColorSensor colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
        NormalizedColorSensor colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor3");


        // Optional: turn on LED if available
        if (colorSensor0 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor0).enableLight(true);
        }
        if (colorSensor1 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor1).enableLight(true);
        }
        if (colorSensor2 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor2).enableLight(true);
        }
        if (colorSensor3 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor3).enableLight(true);
        }

        // Set a fixed gain to start (you can tweak this)
        float gain = 3.0f;
        colorSensor0.setGain(gain);
        colorSensor1.setGain(gain);
        colorSensor2.setGain(gain);
        colorSensor3.setGain(gain);


        // HSV array: [0] = Hue, [1] = Saturation, [2] = Value
        float[] hsv = new float[3];


        int greenCount = 0;
        int purpleCount = 0;
        int patternCount = 0;
        String lastCountedColor = "NONE";




        telemetry.addLine("Color Test: Green / Purple / Pattern");
        telemetry.addData("Rule", "Pattern = 2 PURPLE + 1 GREEN (any order). Max 2P, 1G; extras ignored.");
        telemetry.addData("Gain", gain);


        telemetry.update();




        waitForStart();


        while (opModeIsActive()) {

            String c0 = detectFromSensor(colorSensor0, hsv);
            String c1 = detectFromSensor(colorSensor1, hsv);
            String c2 = detectFromSensor(colorSensor2, hsv);
            String c3 = detectFromSensor(colorSensor3, hsv);
            String detected = combineVirtualColor(c0,c1,c2,c3);

            boolean isGreen = "GREEN".equals(detected);
            boolean isPurple = "PURPLE".equals(detected);
            boolean isTarget = isGreen || isPurple;

            // Read normalized color values (0.0–1.0)
            //NormalizedRGBA colors = colorSensor.getNormalizedColors();



            // Convert to HSV


            //String detected = classifyColor(hue, sat, val);



            if (!isTarget){
                lastCountedColor = "NONE";


            } else if(!detected.equals(lastCountedColor)) {


                if (isGreen){
                    if (greenCount >= 1){
                        telemetry.addLine("No, 1 green done");
                        lastCountedColor = detected;
                    } else {
                        greenCount++;
                        lastCountedColor = detected;
                    }
                } else if (isPurple){
                    if (purpleCount >= 2){
                        telemetry.addLine("No, 2 purple done");
                        lastCountedColor = detected;
                    } else {
                        purpleCount++;
                        lastCountedColor = detected;
                    }
                }


            }


            if (purpleCount == 2 && greenCount==1){
                patternCount++;
                telemetry.addLine("pattern complete");
                telemetry.addData("Total patterns", patternCount);




                purpleCount = 0;
                greenCount = 0;


            }


            // Telemetry output

            telemetry.addData("S0", c0);
            telemetry.addData("S1", c1);
            telemetry.addData("S2", c2);
            telemetry.addData("S3", c3);
            telemetry.addData("Virtual Detected", detected);

            telemetry.addData("Purple Count", purpleCount);
            telemetry.addData("Green Count", greenCount);
            telemetry.addData("Pattern Count", patternCount);
            telemetry.update();


            // Small pause so it doesn't spam insanely fast
            sleep(0);
        }
    }
    private void enableLedIfPossible(NormalizedColorSensor sensor) {
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(true);
        }
    }
    private String detectFromSensor(NormalizedColorSensor sensor, float[] hsv) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsv);
        return classifyColor(hsv[0], hsv[1], hsv[2]);
    }
    private String combineVirtualColor(String c0, String c1, String c2, String c3) {
        boolean anyPurple =
                "PURPLE".equals(c0) || "PURPLE".equals(c1) || "PURPLE".equals(c2) || "PURPLE".equals(c3);
        boolean anyGreen =
                "GREEN".equals(c0) || "GREEN".equals(c1) || "GREEN".equals(c2) || "GREEN".equals(c3);

        if (anyPurple) return "PURPLE";
        if (anyGreen) return "GREEN";
        return "UNKNOWN";
    }
    /**
     * Very simple classification:
     *  - If it's too dark or not colorful enough → UNKNOWN
     *  - Hue in green range → GREEN
     *  - Hue in purple range → PURPLE
     */
    private String classifyColor(float hue, float sat, float val) {
        // Filter out very dark or washed-out readings
        if (val < 0.03 || sat < 0.2) {
            return "UNKNOWN";
        }


        // Green-ish: ~60°–180°
        if (hue >= 150 && hue <= 170) {
            return "GREEN";
        }


        // Purple-ish: roughly magenta / violet zones ~260°–320°
        if (hue >= 220 && hue <= 260) {
            return "PURPLE";
        }


        // Anything else is currently unknown
        return "UNKNOWN";
    }
}



