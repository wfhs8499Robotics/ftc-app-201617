package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Autonomous(name = "AutoLightSensor", group = "Autonomous OpMode")
//@Override
public class AutoLightSensor extends LinearOpMode {
    static final double     WHITE_THRESHOLD = 0.55;  // spans between 0.1 - 0.5 from dark to light
    static final double     APPROACH_SPEED  = 0.15;

    public void runOpMode() throws InterruptedException {
        DcMotor leftmotor = null;
        DcMotor rightmotor = null;
        LightSensor lightSensor;      // Primary LEGO Light sensor,
        TouchSensor lefttouchSensor;  // Hardware Device Object
        TouchSensor righttouchSensor;  // Hardware Device Object
        double savedlightvalue = 0;
        char lastdirection = 'L';

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
        telemetry.addData(">", "Started - going straight for 1.5 sec");
        telemetry.update();
        leftmotor.setPower(0.75d);
        rightmotor.setPower(0.75d);
        sleep(1500);
      /*   telemetry.addData(">", "right 90");
        telemetry.update();
        leftmotor.setPower(1.0d);
        rightmotor.setPower(0d);
        sleep(825);
        telemetry.addData(">", "straight .75 sec");
        telemetry.update();
        leftmotor.setPower(1.0d);
        rightmotor.setPower(1.0d);
        sleep(1500);
     */
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
        sleep(0500);

        leftmotor.setPower(-0.05);
        rightmotor.setPower(-0.05);

        // run until the white line is seen OR the driver presses STOP or it touches the wall;
        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  lightSensor.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        sleep(0500);
        savedlightvalue = lightSensor.getLightDetected();
        leftmotor.setPower(0.05);
        rightmotor.setPower(0.05);
        while (opModeIsActive() && (!lefttouchSensor.isPressed() || !righttouchSensor.isPressed())) {

            while (opModeIsActive() && savedlightvalue <= lightSensor.getLightDetected() && lastdirection == 'R') {
                leftmotor.setPower(0.0);
                rightmotor.setPower(0.10);
                savedlightvalue = lightSensor.getLightDetected();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            lastdirection = 'L';
            while (opModeIsActive() && savedlightvalue <= lightSensor.getLightDetected() && lastdirection == 'L') {
                leftmotor.setPower(0.10);
                rightmotor.setPower(0.0);
                savedlightvalue = lightSensor.getLightDetected();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            lastdirection = 'R';
            leftmotor.setPower(0.05);
            rightmotor.setPower(0.05);
            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  lightSensor.getLightDetected());
            telemetry.addData("last turn : ", lastdirection);
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);

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
            leftmotor.setPower(0.05);
            rightmotor.setPower(-0.05);
            idle();
        }
        while(opModeIsActive() && lefttouchSensor.isPressed() && !righttouchSensor.isPressed()){
            leftmotor.setPower(-0.05);
            rightmotor.setPower(0.05);
            idle();
        }

        // Stop all motors
        leftmotor.setPower(0);
        rightmotor.setPower(0);
    }
}