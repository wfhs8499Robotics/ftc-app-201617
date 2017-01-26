package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Mike on 12/6/2016.
 */
@Autonomous(name = "AutoServo", group = "Autonomous OpMode")
public class AutoServo extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   0;     // period of each cycle
    static final double MAX_POS     =  0.70;     // Maximum rotational position
    static final double MIN_POS     =  0.05;     // Minimum rotational position

    // Define class members
    Servo leftservo;
    Servo rightservo;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;


    @Override
    public void runOpMode() throws InterruptedException {
        // Change the text in quotes to match any servo name on your robot.

        leftservo = hardwareMap.servo.get("left button pusher");
        rightservo = hardwareMap.servo.get("right button pusher");
        leftservo.setDirection(Servo.Direction.REVERSE);

        leftservo.setPosition(MIN_POS);
        rightservo.setPosition(MIN_POS);

        sleep(2000);

        leftservo.setPosition(position);
        rightservo.setPosition(position);

        sleep(2000);

        leftservo.setPosition(MAX_POS);
        rightservo.setPosition(MAX_POS);


        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){

            // slew the servo, according to the rampUp (direction) variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            leftservo.setPosition(position);
            rightservo.setPosition(position);
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
