package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.graphics.Color;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BlueVuforia", group = "Autonomous OpMode")
@Disabled
//@Override
public class BlueVuforia extends LinearOpMode {
    /* Declare OpMode members. */

    public static final String TAG = "BlueVuforia"; // String for logging
    OpenGLMatrix lastLocation = null;
    int i;
    //  Debug program / display messages- yes or no
    boolean debugFlag = false;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    private ElapsedTime runtime = new ElapsedTime();  // used for timing of the encoder run

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     WHEEL_DIAMETER_MM       = 101.6 ;   // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_MM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    DcMotor leftmotor = null; // Hardware Device Object
    DcMotor rightmotor = null; // Hardware Device Object
//    TouchSensor lefttouchSensor = null;  // Hardware Device Object
//    TouchSensor righttouchSensor = null;  // Hardware Device Object
    ColorSensor colorSensor = null;    // Hardware Device Object

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.70;     // Maximum rotational position
    static final double MIN_POS     =  0.05;     // Minimum rotational position

    // Define class members

    double  position = ((MAX_POS - MIN_POS) / 2) + MIN_POS; // Start at halfway position
    boolean rampUp = true;
    boolean bBlueSide = false;
    boolean bRedSide = false;
    Servo leftservo = null; // Hardware Device Object
    Servo rightservo = null; // Hardware Device Object
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};
    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;
    // variable to turn on and off the LED on the Color sensor
    boolean bLedOn = false;

    public void runOpMode() throws InterruptedException {
        leftmotor = hardwareMap.dcMotor.get("left motor");
        leftmotor.setDirection(DcMotor.Direction.REVERSE);
        rightmotor = hardwareMap.dcMotor.get("right motor");
//        lefttouchSensor = hardwareMap.touchSensor.get("left touch sensor");
//        righttouchSensor = hardwareMap.touchSensor.get("right touch sensor");
        leftservo = hardwareMap.servo.get("left button pusher");
        rightservo = hardwareMap.servo.get("right button pusher");
        leftservo.setDirection(Servo.Direction.REVERSE);
        leftservo.setPosition(MIN_POS);
        rightservo.setPosition(MIN_POS);

        int counter = 0;
        /**
         * Start up Vuforia, telling it the id of the view that we wish to use as the parent for
         * the camera monitor feedback; if no camera monitor feedback is desired, use the parameterless
         * constructor instead. We also indicate which camera on the RC that we wish to use. For illustration
         * purposes here, we choose the back camera; for a competition robot, the front camera might
         * prove to be more convenient.
         *
         * Note that in addition to indicating which camera is in use, we also need to tell the system
         * the location of the phone on the robot; see phoneLocationOnRobot below.
         *
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * Vuforia will not load without a valid license being provided. Vuforia 'Development' license
         * keys, which is what is needed here, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Valid Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string form of the key from the Vuforia web site
         * and paste it in to your code as the value of the 'vuforiaLicenseKey' field of the
         * {@link Parameters} instance with which you initialize Vuforia.
         */

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Adzda7//////AAAAGUpLKy6pMUL/jHraDyelv1GA1V8dkeOGLZerg9xBjHAW7cFALsf5Y+T8b6ncYMV+CBje6cvECbbmiEejmvrqDxC0W38xIZN9vyMNGne7E+GXMwVf5gJlg4XTmI38GZ2S4d8y2/qqglzoMDElNJJA7Az97KS84DI+6odaFViAUnvfc4dX5aSX4h0KmBqRUH9761EGNsnz2IQz5/tYAAs9hsMDsBk/fSadAT9NZoc/4l5iJKwlVhKk7avboqJcQx0yzVIUwyjwdCco4SMX+EhSkDuOyaUUK8odF2DOWmsA+x491hpM1qnrGX6XlAJ5npFNQO+pH5D5vNn2HtcUm/a882RCJ0Vu6BasSJrdgJugiway";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        /**
         * Load the data sets that for the trackable objects we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project' view over there on the left of the screen). You can make your own datasets
         * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager. PDFs for the
         * example "FTCImages", datasets can be found in in this project in the
         * documentation directory.
         */
        VuforiaTrackables FTCImages = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable blueMiddle = FTCImages.get(0);
        blueMiddle.setName("BlueMiddle");  // wheels

        VuforiaTrackable redCorner = FTCImages.get(1);
        redCorner.setName("RedCorner");  // tools

        VuforiaTrackable blueCorner = FTCImages.get(2);
        blueCorner.setName("BlueCorner");  // legos

        VuforiaTrackable redMiddle = FTCImages.get(3);
        redMiddle.setName("RedMiddle");  // Gears

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(FTCImages);
        // Start tracking the data sets we care about.
        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener) FTCImages.get(0).getListener();
        VuforiaTrackableDefaultListener tools  = (VuforiaTrackableDefaultListener) FTCImages.get(1).getListener();
        VuforiaTrackableDefaultListener legos  = (VuforiaTrackableDefaultListener) FTCImages.get(2).getListener();
        VuforiaTrackableDefaultListener gears  = (VuforiaTrackableDefaultListener) FTCImages.get(3).getListener();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        // Reset the encoders  and  wait for a short amount of time
        leftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        // Set the motors to encoder mode
        leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // get a reference to our ColorSensor object.
        telemetry.addData("Status", "Done - Resetting Encoders");
        telemetry.update();
        colorSensor = hardwareMap.colorSensor.get("color sensor");
        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                leftmotor.getCurrentPosition(),
                rightmotor.getCurrentPosition());
        // tell vuforia to start to track the images
        FTCImages.activate();

        // tell the driver everything is ready to go
        telemetry.addData(">", "Finished initiating, Vuforia ready, press play to start moving");
        telemetry.update();
        // wait for the game to begin
        waitForStart();
//        encoderDrive(DRIVE_SPEED,  71,  71, 8.0);  // S1: forward 64 Inches with 5 Sec timeout
//        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(TURN_SPEED,   78,  78, 8.0);  // S3: forward 12 Inches with 4 Sec timeout

        // set the motors to encoder mode
        leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // full steam ahead
//        leftmotor.setPower(0.6);
//        rightmotor.setPower(0.6);
        // until we find an image in the camera from vuforia
        while (opModeIsActive() && legos.getRawPose() == null) { //legos = bluecorner, tools = redcorner
        //slow down the look so we dont go hyper
            idle();
        }
        // found an image now stop.
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        //Tell the driver we see an image
        if (debugFlag)  {
            telemetry.addData("sees image", null);
            telemetry.update();
        }

        // mark what side we are on
        if (legos.getRawPose() != null) {
            bBlueSide = true;
            goToImagePushButton(legos);
        }
        encoderDrive(DRIVE_SPEED,  -6,  -6, 5.0);  // S1: Backward 5 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   -6, 6, 4.0);  // S2: Turn Left 6 Inches with 4 Sec timeout or 45 degrees
        encoderDrive(DRIVE_SPEED, -68, -68, 10.0);  // S3: Reverse 68 Inches with 10 Sec timeout
        encoderDrive(TURN_SPEED,   6, -6, 4.0);  // S2: Turn right 6 Inches with 4 Sec timeout or 45 degrees
        // go to the other image on our side
        goToImagePushButton(wheels);
        encoderDrive(DRIVE_SPEED,  -60,  -60, 8.0);  // S1: Backward 5 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   6, -6, 4.0);  // S2: Turn right 6 Inches with 4 Sec timeout or 45 degrees
//        encoderDrive(DRIVE_SPEED, -72, -72, 10.0);  // S3: Reverse 68 Inches with 10 Sec timeout
    }
/*
* based in the image object that is passed in calculate angles and go to the image.  once you arrive at the image
* determine the color of the right beacon and press the appropriate button.
*/
    private void goToImagePushButton(VuforiaTrackableDefaultListener myImage) throws InterruptedException {
        // get the angle to the image
        VectorF angles = anglesFromTarget(myImage);
        if (debugFlag){
            telemetry.addData("angles = ", angles);
            telemetry.update();
            sleep(2000);
        }

        // figure out how many degrees to turn
        VectorF trans = navOffWall(myImage.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(0, 0, 500));
        // adjust to the right angle to go
        if (debugFlag){
            telemetry.addData("trans = ", trans);
            telemetry.update();
            sleep(2000);
        }
        if(trans.get(0) > 0) {
            leftmotor.setPower(0.05);
            rightmotor.setPower(-0.05);
        } else {
            leftmotor.setPower(-0.05);
            rightmotor.setPower(0.05);
        }
        // using the current position keep going until it matches where we want to be
        do {
            if (myImage.getPose() != null) {
                trans = navOffWall(myImage.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(0, 0, 500));
//                if (debugFlag){
                    telemetry.addData("tran 0 = ", Math.abs(trans.get(0)));
                    telemetry.addData("degrees = ", Math.toDegrees(angles.get(0)) - 90);
                    telemetry.update();
//                }
            }
            idle();
        } while (opModeIsActive() && Math.abs(trans.get(0)) > 30);
        //Stop!!  We are at the right angle
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        // tell the driver
        if (debugFlag){
            telemetry.addData("trans = ", trans);
            telemetry.update();
            sleep(1000);
        }
/*        // we are at the right angle now to walk to the position
        // now go to in front of the image using run to position
        leftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // set the target position for each motor
        if (debugFlag){
            telemetry.addData("current = ", leftmotor.getCurrentPosition());
            telemetry.addData("tran.get 0 = ", trans.get(0));
            telemetry.addData("tran.get 2 = ", trans.get(2));
            telemetry.addData("Hypoten = ", Math.hypot(trans.get(0), trans.get(2)) + 150);
            telemetry.addData("wheel rotations to get there", (Math.hypot(trans.get(0), trans.get(2)) + 150) / 319.186);
            telemetry.addData("ticks to get there", ((Math.hypot(trans.get(0), trans.get(2)) + 150) / 319.186) * 1440);
            telemetry.addData("new position = ", ((int) (leftmotor.getCurrentPosition() + (((Math.hypot(trans.get(0), trans.get(2)) + 150) / 319.186) * 1440))));

            telemetry.update();
            sleep(1000);
        }

        leftmotor.setTargetPosition((int) (leftmotor.getCurrentPosition() + ((Math.hypot(trans.get(0), trans.get(2)) + 150) / (319.186 *1440))));
        rightmotor.setTargetPosition((int) (rightmotor.getCurrentPosition() + ((Math.hypot(trans.get(0), trans.get(2)) + 150) / (319.186 *1440))));
        //just a little power to get there
        if (debugFlag){
            telemetry.addData("new position = ", ((int) (leftmotor.getCurrentPosition() + (((Math.hypot(trans.get(0), trans.get(2)) + 150) / 319.186) *1440))));
            telemetry.update();
            sleep(1000);
        }

        leftmotor.setPower(0.15);
        rightmotor.setPower(0.15);
        // while we are moving.. just wait
        while(opModeIsActive() && leftmotor.isBusy() && rightmotor.isBusy()){
          idle();
        }
        if (debugFlag){
            telemetry.addData("trans = ", trans);
            telemetry.update();
            sleep(1000);
        }
        */
        encoderDriveMM(0.4, Math.hypot(trans.get(0), trans.get(2)) + 0, Math.hypot(trans.get(0), trans.get(2)) + 0, 3);  //was + 150 for the distance
        // there!
        // turn to get get robot square with the images
        while (opModeIsActive() && (myImage.getPose() == null || Math.abs(myImage.getPose().getTranslation().get(0)) > 5)){
            if (debugFlag) {
                telemetry.addData("close abs get 0 = ", Math.abs(myImage.getPose().getTranslation().get(0)));
                telemetry.update();
                sleep(1000);
            }
            if(myImage.getPose() != null) {
                if (myImage.getPose().getTranslation().get(0) > 5) {
                    leftmotor.setPower(0.03);
                    rightmotor.setPower(-0.03);
                } else {
                    leftmotor.setPower(-0.03);
                    rightmotor.setPower(0.03);
                }
            } else {
                    leftmotor.setPower(0.03);
                    rightmotor.setPower(-0.03);
                }
            idle();
        }
        //Stop..  we are right in front of the image
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        //Move forward to be able to push the buttons
        encoderDrive(0.3,  8,  8, 5.0);  // S1: Forward 8 Inches with 5 Sec timeout
        // Tell the driver
        telemetry.addData("stopping", null);
        telemetry.update();
        // check the color sensor to see what button to press
        // convert the RGB values to HSV values.
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        // send the info back to driver station using telemetry function.
        telemetry.addData("LED", bLedOn ? "On" : "Off");
//        telemetry.addData("Clear", colorSensor.alpha());
//        telemetry.addData("Red  ", colorSensor.red() * 8);
//        telemetry.addData("Green", colorSensor.green() * 8);
//        telemetry.addData("Blue ", colorSensor.blue() * 8);
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", hsvValues[1]);
        telemetry.addData("Value", hsvValues[2]);
        // TODO based on the side we are on red or blue and the color of the right side of the beacon..
        if (hsvValues[0] > 100){ // on blue side and hue > 100 is blue
            // push the button on the left
            rightservo.setPosition(MAX_POS);
            // allow the servo to move
            sleep(CYCLE_MS);
        }
        if (hsvValues[0] < 100){ // on blue side and hue < 100 is red
            // push the button on the right
            leftservo.setPosition(MAX_POS);
            // allow the servo to move
            sleep(CYCLE_MS);
        }
        // reset to neutral position
        leftservo.setPosition(MIN_POS);
        rightservo.setPosition(MIN_POS);
    }

    /**
 * A simple utility that extracts positioning information from a transformation matrix
 * and formats it in a form palatable to a human being.
 */
String format(OpenGLMatrix transformationMatrix) {
    return transformationMatrix.formatAsTransform();
}

public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
    if (debugFlag){
        telemetry.addData("now - trans = ", trans);
        telemetry.addData("now - robot angle = ", robotAngle);
        telemetry.addData("now - off wall", offWall);
        telemetry.update();
        sleep(1000);
    }

    return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
}

public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){
    float [] data = image.getRawPose().getData();
    float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
    if (debugFlag){
        telemetry.addData("aft data = ", data);
        telemetry.addData("aft rotation = ", rotation);
        telemetry.update();
        sleep(1000);
    }

    double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
    double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
    double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
    return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
}

/*
 *  Method to perform a relative move, based on encoder counts.
 *  Encoders are not reset as the move is based on the current position.
 *  Move will stop if any of three conditions occur:
 *  1) Move gets to the desired position
 *  2) Move runs out of time
 *  3) Driver stops the opmode running.
 *
 *  Note: Reverse movement is obtained by setting a negative distance (not speed)
 *      encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 48 Inches with 5 Sec timeout
 *      encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
 *      encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
 *
 */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftmotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightmotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftmotor.setTargetPosition(newLeftTarget);
            rightmotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftmotor.setPower(Math.abs(speed));
            rightmotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftmotor.isBusy() && rightmotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftmotor.getCurrentPosition(),
                        rightmotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftmotor.setPower(0);
            rightmotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    /*
*  Method to perform a relative move, based on encoder counts.
*  Encoders are not reset as the move is based on the current position.
*  Move will stop if any of three conditions occur:
*  1) Move gets to the desired position
*  2) Move runs out of time
*  3) Driver stops the opmode running.
*
*  Note: Reverse movement is obtained by setting a negative distance (not speed)
*      encoderDrive(DRIVE_SPEED,  480,  480, 5.0);  // S1: Forward 48 MMs with 5 Sec timeout
*      encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 MMs with 4 Sec timeout
*      encoderDrive(DRIVE_SPEED, -240, -240, 4.0);  // S3: Reverse 24 MMs with 4 Sec timeout
*
*/
    public void encoderDriveMM(double speed,
                               double leftMMs, double rightMMs,
                               double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftmotor.getCurrentPosition() + (int)(leftMMs * COUNTS_PER_MM);
            newRightTarget = rightmotor.getCurrentPosition() + (int)(rightMMs * COUNTS_PER_MM);
            leftmotor.setTargetPosition(newLeftTarget);
            rightmotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftmotor.setPower(Math.abs(speed));
            rightmotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftmotor.isBusy() && rightmotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftmotor.getCurrentPosition(),
                        rightmotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftmotor.setPower(0);
            rightmotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}

