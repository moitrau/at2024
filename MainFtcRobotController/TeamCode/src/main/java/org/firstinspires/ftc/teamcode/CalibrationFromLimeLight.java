package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.hardware.LimelightHelpers.Results;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */

//@TeleOp(name = "TestLimeLight3 (Blocks to Java)")
public class CalibrationFromLimeLight  {

    private Limelight3A limelight;
    private double xCordinate;
    private double yCordinate;
    private double yawAngleRobot;
    private double marginOfError = 0.5;
    public double resultCordinates[] = new double[3];
    HardwareMap hardwareMap;

    public void LimeLight3A(double InputXCordinate, double InputYCordinate,
                            double InputYawAngle, double InputMargin, HardwareMap hMap)
    {
        hardwareMap = hMap;
        xCordinate = InputXCordinate;
        yCordinate = InputYCordinate;
        yawAngleRobot = InputYawAngle;
        if (InputMargin != marginOfError){
            marginOfError = InputMargin;
        }
    }

    public double [] CalibratePoseWithLimelIght()
    {
        double[] xvalues = new double[20];
        double[] yvalues = new double[20];
        double[] yawvalues = new double[20];
        double xMean = 0;
        double yMean = 0;
        double yawMean = 0;
        int count = 0;
        int loop = 0;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        while (loop < 25) {
            LLStatus status = limelight.getStatus();
            resultCordinates[0] = 0;
            resultCordinates[1] = 0;
            resultCordinates[2] = 0;

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose();

                if (result.isValid()) {

                    YawPitchRollAngles orientation = botpose.getOrientation();

                    xvalues[count] = botpose.getPosition().x * 39.3701;
                    yvalues[count] = botpose.getPosition().y * 39.3701;
                    yawvalues[count] = orientation.getYaw();

                    count++;

                    if (count == xvalues.length - 1){

                        xMean = getFinalMean(xvalues);
                        yMean = getFinalMean(yvalues);
                        yawMean = getFinalMean(yawvalues);
                        count = 0;

                        for (int i =0; i<=xvalues.length-1; i++) {
                            xvalues[i] = 0;
                            yvalues[i] = 0;
                            yawvalues[i] = 0;
                        }

                        if(Math.abs(xCordinate - xMean) > 0.5){
                            resultCordinates[0] = xMean;
                        }
                        else {
                            resultCordinates[0] = xCordinate;
                        }

                        if(Math.abs(yCordinate - yMean) > 0.5){
                            resultCordinates[1] = yMean;
                        }
                        else {
                            resultCordinates[1] = yCordinate;
                        }

                        if(Math.abs(yawAngleRobot - yawMean) > 0.5){
                            resultCordinates[2] = yawMean;
                        }
                        else {
                            resultCordinates[2] = yawAngleRobot;
                        }

                        return resultCordinates;
                    }

                }
            } else {
                return resultCordinates;
            }

        loop++;

        }
        limelight.stop();

        return resultCordinates;
    }

    public static double getFinalMean (double[] array){
        double mean = calculateMean(array);
        double stdDev = calculateStandardDeviation(array, mean);

        int count = 0;
        double finalSum = 0;

        System.out.println("Mean: " + mean);
        System.out.println("Standard Deviation: " + stdDev);

        // Identify and print outliers (Z-Score > 2)
        System.out.println("Outliers:");
        for (double value : array) {
            double zScore = (value - mean) / stdDev;
            if (Math.abs(zScore) > 2) {  // Threshold of 2 standard deviations
                System.out.println(value + " (Z-Score: " + zScore + ")");
            }
            else {
                finalSum = finalSum + value;
                count++;
            }
        }
        return finalSum/count;
    }
    // Method to calculate the mean
    public static double calculateMean(double[] array) {
        double sum = 0.0;
        for (double num : array) {
            sum += num;
        }
        return sum / array.length;
    }

    // Method to calculate standard deviation
    public static double calculateStandardDeviation(double[] array, double mean) {
        double sum = 0.0;
        for (double num : array) {
            sum += Math.pow(num - mean, 2);
        }
        return Math.sqrt(sum / array.length);
    }
}
