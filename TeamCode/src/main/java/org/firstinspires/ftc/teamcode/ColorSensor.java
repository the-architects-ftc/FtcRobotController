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
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");


        // Optional: turn on LED if available
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }


        // Set a fixed gain to start (you can tweak this)
        float gain = 3.0f;
        colorSensor.setGain(gain);


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


            // Read normalized color values (0.0–1.0)
            NormalizedRGBA colors = colorSensor.getNormalizedColors();


            // Convert to HSV
            Color.colorToHSV(colors.toColor(), hsv);
            float hue = hsv[0];
            float sat = hsv[1];
            float val = hsv[2];
            String detected = classifyColor(hue, sat, val);


            boolean isGreen = "GREEN".equals(detected);
            boolean isPurple = "PURPLE".equals(detected);
            boolean isTarget = isGreen || isPurple;
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

            telemetry.addData("Detected Color", detected);

            telemetry.addData("Purple Count", purpleCount);
            telemetry.addData("Green Count", greenCount);
            telemetry.addData("Pattern Count", patternCount);
            telemetry.update();


            // Small pause so it doesn't spam insanely fast
            sleep(0);
        }
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



