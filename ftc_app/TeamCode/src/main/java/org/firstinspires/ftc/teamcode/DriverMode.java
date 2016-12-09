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
    Servo servo = null;         // Hardware Device Object

    float StickPercent = 0.5f;  // only use 50 percent power as the default speed at full throttle
    // settings for the Servo
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.4;     // Maximum rotational position
    static final double MIN_POS     =  0.1;     // Minimum rotational position
    double  position = ((MAX_POS - MIN_POS) / 2) + MIN_POS; // Start at halfway position
    // all the variables we need
    double left;
    double right;
    float hypermode;
    float seanmode;
    float driveadjustment;
    float pushbeaconright;
    float pushbeaconleft;
    boolean centerservo;
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
        // Get the servo object created
        servo = hardwareMap.servo.get("button pusher");
        //position the servo to center
        servo.setPosition(position);
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
        pushbeaconright = gamepad2.right_trigger;
        pushbeaconleft = gamepad2.left_trigger;
        centerservo = gamepad2.y;
        // move the servo forward on the right
        if (pushbeaconright > 0){
            servo.setPosition(MAX_POS);
        }
        // move the servo forward on the left
        if (pushbeaconleft > 0){
            servo.setPosition(MIN_POS);
        }
        // center the servo
        if (centerservo){
            servo.setPosition(position);
        }
        // set drive adjustment to the default stick percent
        driveadjustment = StickPercent;
        // change the drive adjustment for hypermode
        if (hypermode > 0){
            driveadjustment = StickPercent * 2.0f;
        }
        // change the drive adjustment to slow mode
        if (seanmode > 0){
            driveadjustment = StickPercent * 0.5f;
        }
        // set the power of the motor to the stick value multiplied by the adjustment
        leftmotor.setPower(left * driveadjustment);
        rightmotor.setPower(right * driveadjustment);
        // Tell the driver
        telemetry.addData("left",  "%.2f", left * driveadjustment);
        telemetry.addData("right", "%.2f", right * driveadjustment);
        if (pushbeaconright > 0){
            telemetry.addData("servo", "servo right pushed %.2f", MAX_POS);
        }
        if (pushbeaconleft > 0){
            telemetry.addData("servo", "servo left pushed %.2f", MIN_POS);
        }
        if (centerservo){
            telemetry.addData("servo", "servo center pushed %.2f", position);
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
