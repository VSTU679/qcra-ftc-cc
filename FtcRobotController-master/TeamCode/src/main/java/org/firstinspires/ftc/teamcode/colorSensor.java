package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class colorSensor extends LinearOpMode {
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Initialize sensor (same device, 2 interfaces)
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        telemetry.addData("Ready to start", "TestSensor");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();



            //if white is detected
            if (red > 200 && green > 200 && blue > 200) {
                telemetry.addLine("White Detected- Stopping");
            } else {
                telemetry.addData("Not White", "good");
            }
            telemetry.update();
            }
        }
    }
}
