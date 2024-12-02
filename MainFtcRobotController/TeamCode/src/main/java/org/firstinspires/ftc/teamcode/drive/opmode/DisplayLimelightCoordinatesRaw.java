package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CalibrationFromLimeLight;


@TeleOp(name = "Display Limelight Coordinates Raw")
public class DisplayLimelightCoordinatesRaw extends LinearOpMode {

    private Limelight3A limelight;
    private CalibrationFromLimeLight calibrationFromLimeLight;

    @Override
    public void runOpMode() {
        // Initialize Limelight and CalibrationFromLimeLight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        calibrationFromLimeLight = new CalibrationFromLimeLight();

        // Set the initial coordinates and margin of error
        calibrationFromLimeLight.LimeLight3A(0, 0, 0, 0.5, hardwareMap);
        double[] resultCordinates = calibrationFromLimeLight.CalibratePoseWithLimelIghtRaw();

        // Display x, y, and heading using telemetry
        telemetry.addData("X Coordinate", resultCordinates[0]);
        telemetry.addData("Y Coordinate", resultCordinates[1]);
        telemetry.addData("Heading (Yaw)", resultCordinates[2]);
        telemetry.update();
        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Calibrate pose with Limelight
            resultCordinates = calibrationFromLimeLight.CalibratePoseWithLimelIghtRaw();

            // Display x, y, and heading using telemetry
            telemetry.addData("X Coordinate", resultCordinates[0]);
            telemetry.addData("Y Coordinate", resultCordinates[1]);
            telemetry.addData("Heading (Yaw)", resultCordinates[2]);
            telemetry.update();
        }
    }
}