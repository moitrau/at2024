package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Sensor Sample", group = "Sensor")
public class ColorSensorSample extends LinearOpMode {

    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // Read the color sensor values
            int red = colorSensor.red();
            int yellow = (colorSensor.red() + colorSensor.green()) / 2;
            int blue = colorSensor.blue();

            // Read the distance sensor value
            double distance = distanceSensor.getDistance(DistanceUnit.CM);

            // Send the values to the driver station
            telemetry.addData("Red", red);
            telemetry.addData("Yellow", yellow);
            telemetry.addData("Blue", blue);
            telemetry.addData("Distance (cm)", distance);
            telemetry.update();
        }
    }
}
