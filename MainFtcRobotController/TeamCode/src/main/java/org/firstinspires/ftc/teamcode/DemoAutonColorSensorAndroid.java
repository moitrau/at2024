
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@Autonomous(name = "DemoAutonColorSensorAndroid")
public class DemoAutonColorSensorAndroid extends LinearOpMode {

    private ColorSensor colorSensor_REV_ColorRangeSensor;

    /**
     * This OpMode demonstrates the color and distance features of the REV sensor.
     */
    @Override
    public void runOpMode() {
        double distance;
        double hue;

        colorSensor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Display distance info.
                ((NormalizedColorSensor) colorSensor_REV_ColorRangeSensor).setGain(2);
                distance = ((DistanceSensor) colorSensor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM);
                telemetry.addData("Dist to tgt (cm)", ((DistanceSensor) colorSensor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM));
                hue = Double.parseDouble(JavaUtil.formatNumber(JavaUtil.colorToHue(((NormalizedColorSensor) colorSensor_REV_ColorRangeSensor).getNormalizedColors().toColor()), 0));
                telemetry.addData("Hue", Double.parseDouble(JavaUtil.formatNumber(hue, 0)));
                // Use hue to determine if it's red, yellow, blue sample
                if (distance <= 4 && hue >= 200 && hue <= 240) {
                    telemetry.addData("Detection Status", "Blue Sample");
                } else if (distance <= 4 && hue >= 80 && hue <= 100) {
                    telemetry.addData("Detection Status", "Yellow Sample");
                } else if (distance <= 4 && hue >= 0 && hue <= 60) {
                    telemetry.addData("Detection Status", "Red Sample");
                } else {
                    telemetry.addData("Detection Status", "No Sample Detected !  Sample distance should be less than 4 cms from the sensor.");
                }
                telemetry.update();
            }
        }
    }
}