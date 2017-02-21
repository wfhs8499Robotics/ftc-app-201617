package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * The code is structured as an Iterative OpMode
 */

@TeleOp(name="8944: Simple Teleop", group="TeleOp")

public class DriverMode extends OpMode {

    DcMotor leftmotor = null;   // Hardware Device Object
    DcMotor rightmotor = null;  // Hardware Device Object
    DcMotor liftmotor = null;   // Hardware Device Object
    Servo leftservo = null;         // Hardware Device Object
    Servo rightservo = null;         // Hardware Device Object
    Servo liftrelease = null;         // Hardware Device Object

    float LiftPercent = 0.5f;  // Lift Motor:: only use 50 percent power as the default speed at full throttle
    float StickPercent = 0.5f;  // only use 50 percent power as the default speed at full throttle
    // settings for the Servo
    static final double MAX_POS     =  0.70;     // Maximum rotational position
    static final double MIN_POS     =  0.05;     // Minimum rotational position
    double  position = ((MAX_POS - MIN_POS) / 2) + MIN_POS; // Start at halfway position
    // settings for the lift release servo
    static final double LIFT_MAX_POS     =  0.70;     // Maximum rotational position
    static final double LIFT_MIN_POS     =  0.05;     // Minimum rotational position

    // all the variables we need
    double left;
    double right;
    double lift;
    float hypermode;
    float seanmode;
    float hyperliftmode;
    float seanliftmode;
    float driveadjustment;
    float liftadjustment;
    boolean pushbeaconright = false;
    boolean pushbeaconleft = false;
    boolean centerservo = false;
    boolean extendbothservo = false;
    boolean liftreleasepushed = false;
    boolean bSeanMode = false;
    boolean bFastMode = false;
    boolean bSeanButtonPushed = false;
    boolean bFastButtonPushed = false;
    boolean bSeanLiftMode = false;
    boolean bFastLiftMode = false;
    boolean bSeanLiftButtonPushed = false;
    boolean bFastLiftButtonPushed = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        // get the motor objects created
        leftmotor = hardwareMap.dcMotor.get("left motor");
        leftmotor.setDirection(DcMotor.Direction.REVERSE);
        rightmotor = hardwareMap.dcMotor.get("right motor");
        // get the motor objects created
        liftmotor = hardwareMap.dcMotor.get("lift");
        // Get the servo object created
        leftservo = hardwareMap.servo.get("left button pusher");
        rightservo = hardwareMap.servo.get("right button pusher");
        leftservo.setDirection(Servo.Direction.REVERSE);
        //position the servo to the minimum position
        leftservo.setPosition(MIN_POS);
        rightservo.setPosition(MIN_POS);
        // Get the lift release servo object created
        liftrelease = hardwareMap.servo.get("left button pusher");
//        liftrelease.setDirection(Servo.Direction.REVERSE);
        //position the servo to Minimum position
        liftrelease.setPosition(LIFT_MIN_POS);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver - I am ready");    //
        updateTelemetry(telemetry);
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
// nothing to do here
    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // nothing to do here
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        // get all the gamepad variables
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        hypermode = gamepad1.right_trigger;
        seanmode = gamepad1.left_trigger;
        pushbeaconright = gamepad2.dpad_right;
        pushbeaconleft = gamepad2.dpad_left;
        lift = -gamepad2.left_stick_y;
        centerservo = gamepad2.dpad_down;
        extendbothservo = gamepad2.dpad_up;
        hyperliftmode = gamepad2.right_trigger;
        seanliftmode = gamepad2.left_trigger;
        liftreleasepushed = gamepad2.y;
        // if either trigger has started to be pushed, wait til it goes to 0 to toggle modes
        if (hypermode > 0){
            bFastButtonPushed = true;
        }
        if (hypermode == 0 && bFastButtonPushed){
            bFastButtonPushed = false;
            bFastMode = !bFastMode;
            if (bFastMode){
                bSeanMode = false;
            }
        }
        if (seanmode > 0){
            bSeanButtonPushed = true;
        }
        if (seanmode == 0 && bSeanButtonPushed){
            bSeanButtonPushed = false;
            bSeanMode = !bSeanMode;
            if (bSeanMode){
                bFastMode = false;
            }
        }
        // Lift Motor Controls:: if either trigger has started to be pushed, wait til it goes to 0 to toggle modes
        if (hyperliftmode > 0){
            bFastLiftButtonPushed = true;
        }
        if (hyperliftmode == 0 && bFastLiftButtonPushed){
            bFastLiftButtonPushed = false;
            bFastLiftMode = !bFastLiftMode;
            if (bFastLiftMode){
                bSeanLiftMode = false;
            }
        }
        if (seanliftmode > 0){
            bSeanLiftButtonPushed = true;
        }
        if (seanliftmode == 0 && bSeanLiftButtonPushed){
            bSeanLiftButtonPushed = false;
            bSeanLiftMode = !bSeanLiftMode;
            if (bSeanLiftMode){
                bFastLiftMode = false;
            }
        }
        // move the servo forward on the right
        if (pushbeaconright == true){
            rightservo.setPosition(MAX_POS);
            leftservo.setPosition(MIN_POS);
        }
        // move the servo forward on the left
        if (pushbeaconleft == true){
            leftservo.setPosition(MAX_POS);
            rightservo.setPosition(MIN_POS);
        }
        // center the servo
        if (centerservo){
            leftservo.setPosition(MIN_POS);
            rightservo.setPosition(MIN_POS);
        }
        // Extend both servos
        if (extendbothservo){
            leftservo.setPosition(MAX_POS);
            rightservo.setPosition(MAX_POS);
        }
        if (liftreleasepushed) {
            liftrelease.setPosition(LIFT_MAX_POS);
        }
        // set drive adjustment to the default stick percent
        driveadjustment = StickPercent;
        // change the drive adjustment for hypermode
        if (bFastMode){
            driveadjustment = StickPercent * 2.0f;
        }
        // change the drive adjustment to slow mode
        if (bSeanMode){
            driveadjustment = StickPercent * 0.5f;
        }
        // Lift Motor::  set drive adjustment to the default stick percent
        liftadjustment = LiftPercent;
        // change the drive adjustment for hypermode
        if (bFastLiftMode){
            liftadjustment = LiftPercent * 2.0f;
        }
        // change the drive adjustment to slow mode
        if (bSeanLiftMode){
            liftadjustment = LiftPercent * 0.5f;
        }

        // set the power of the motor to the stick value multiplied by the adjustment
        leftmotor.setPower(left * driveadjustment);
        rightmotor.setPower(right * driveadjustment);
        liftmotor.setPower(lift * liftadjustment);

        // Tell the driver
        telemetry.addData("Fast Mode", bFastMode);
        telemetry.addData("Sean Mode", bSeanMode);
        telemetry.addData("left",  "%.2f", left * driveadjustment);
        telemetry.addData("right", "%.2f", right * driveadjustment);
        telemetry.addData("Lift Fast Mode", bFastLiftMode);
        telemetry.addData("Lift Sean Mode", bSeanLiftMode);
        telemetry.addData("Lift",  "%.2f", lift * liftadjustment);

        if (pushbeaconright == true){
            telemetry.addData("servo", "servo right pushed %.2f", MAX_POS);
        }
        if (pushbeaconleft == true){
            telemetry.addData("servo", "servo left pushed %.2f", MIN_POS);
        }
        if (centerservo){
            telemetry.addData("servo", "servo center pushed %.2f", MIN_POS);
        }
        if (extendbothservo){
            telemetry.addData("servo", "servo extend pushed %.2f", MAX_POS);
        }
        updateTelemetry(telemetry);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // nothing to do here
    }
}
