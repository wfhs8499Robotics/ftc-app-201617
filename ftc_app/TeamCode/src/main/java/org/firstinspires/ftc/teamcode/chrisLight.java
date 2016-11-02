package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Autonomous(name = "LightSensor Test", group = "Autonomous OpMode")
//@Override
public class chrisLight extends LinearOpMode {
    static final double     WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    static final double     APPROACH_SPEED  = 0.5;

    public void runOpMode() throws InterruptedException {
        DcMotor leftmotor = null;
        DcMotor rightmotor = null;
        LightSensor lightSensor;      // Primary LEGO Light sensor,
        TouchSensor lefttouchSensor;  // Hardware Device Object
        TouchSensor righttouchSensor;  // Hardware Device Object
        double savedlightvalue = 0;
        char lastdirection = 'R';

        leftmotor = hardwareMap.dcMotor.get("left motor");
        leftmotor.setDirection(DcMotor.Direction.REVERSE);
        rightmotor = hardwareMap.dcMotor.get("right motor");
        // get a reference to our Light Sensor object.
        lightSensor = hardwareMap.lightSensor.get("light sensor");                // Primary LEGO Light Sensor
        lefttouchSensor = hardwareMap.touchSensor.get("left touch sensor");
        righttouchSensor = hardwareMap.touchSensor.get("right touch sensor");
        int counter = 0;
        //  lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.
        telemetry.addData(">", "Finished initiating, press play to start moving");
        telemetry.addData("Light Level", lightSensor.getLightDetected());
        telemetry.update();

        waitForStart();
        // turn on LED of light sensor.
        lightSensor.enableLed(true);

        // Start the robot moving forward, and then begin looking for a white line.
        leftmotor.setPower(APPROACH_SPEED);
        rightmotor.setPower(APPROACH_SPEED);

        // run until the white line is seen OR the driver presses STOP or it touches the wall;
        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  lightSensor.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        // TODO: we have found the line now walk up it to the Beacon
        while (opModeIsActive() && (lightSensor.getLightDetected() >= WHITE_THRESHOLD) && (!lefttouchSensor.isPressed() || !righttouchSensor.isPressed())) {

            if (savedlightvalue < lightSensor.getLightDetected() && lastdirection == 'R') {
                leftmotor.setPower(0.1);
                rightmotor.setPower(0.2);
                lastdirection = 'L';
            }
            if (savedlightvalue < lightSensor.getLightDetected() && lastdirection == 'L') {
                leftmotor.setPower(0.2);
                rightmotor.setPower(0.1);
                lastdirection = 'R';
            }
            savedlightvalue = lightSensor.getLightDetected();
            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  lightSensor.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);


        /// && (!lefttouchSensor.isPressed() || !righttouchSensor.isPressed())

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        // send the info back to driver station using telemetry function.
        if (righttouchSensor.isPressed()) {
            telemetry.addData("Touch", "Right Is Pressed");
        }
        if (lefttouchSensor.isPressed()) {
            telemetry.addData("Touch", "Left Is Pressed");
        }
        if (lefttouchSensor.isPressed() && righttouchSensor.isPressed()) {
            telemetry.addData("Touch", "Both are pressed");
        }
        telemetry.update();

        while(opModeIsActive() && righttouchSensor.isPressed() && !lefttouchSensor.isPressed()){
            leftmotor.setPower(0.2);
            rightmotor.setPower(0);
            idle();
        }
        while(opModeIsActive() && lefttouchSensor.isPressed() && !righttouchSensor.isPressed()){
            leftmotor.setPower(0);
            rightmotor.setPower(0.2);
            idle();
        }

        // Stop all motors
        leftmotor.setPower(0);
        rightmotor.setPower(0);
    }
}