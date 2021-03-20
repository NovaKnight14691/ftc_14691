package org.firstinspires.ftc.teamcode.opmode;

        import android.app.Activity;
        import android.graphics.Color;
        import android.view.View;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DistanceSensor;

        import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
        import org.firstinspires.ftc.teamcode.configuration.RobotConfiguration;

        import java.util.Locale;


@TeleOp(name = "Sensor: REVColorDistance", group = "Sensor")
public class RevColorSensorTest extends LinearOpMode {


    @Override
    public void runOpMode() {

        RobotConfiguration robot = new RobotConfiguration();
        robot.init(hardwareMap);
        ColorSensor sensorColor = robot.colorSensor;

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;

        // wait for the start button to be pressed.
        waitForStart();


        while (opModeIsActive()) {

            Color.RGBToHSV((int) (sensorColor.red()),
                    (int) (sensorColor.green()),
                    (int) (sensorColor.blue()),
                    hsvValues);

            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.update();
        }
    }
}
