package org.firstinspires.ftc.teamcode.configuration;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;


public class BestRobotConfiguration
{
    /* Public OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor bleftDrive = null;
    public DcMotor brightDrive = null;
    public DistanceSensor distance = null;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public BestRobotConfiguration() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        DistanceSensor sensorColorRange;


        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        bleftDrive  = hwMap.get(DcMotor.class, "bleft_drive");
        brightDrive = hwMap.get(DcMotor.class, "bright_drive");
        bleftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        brightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        bleftDrive.setPower(0);
        brightDrive.setPower(0);

        distance = hwMap.get(DistanceSensor.class, "distance");


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

