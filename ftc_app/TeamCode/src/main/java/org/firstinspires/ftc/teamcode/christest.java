import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Green Track", group = "Autonomous OpMode")
//@Override
public class christest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        DcMotor leftmotor = null;
        DcMotor rightmotor = null;
        leftmotor = hardwareMap.dcMotor.get("left motor");
        leftmotor.setDirection(DcMotor.Direction.REVERSE);
        rightmotor = hardwareMap.dcMotor.get("right motor");

        telemetry.addData(">", "Finished initiating, press play to start moving");
        telemetry.update();

        waitForStart();
        telemetry.addData(">", "Started - going straight for 3 sec");
        telemetry.update();
        leftmotor.setPower(1.0d);
        rightmotor.setPower(1.0d);
        sleep(3000);
        telemetry.addData(">", "right 90");
        telemetry.update();
        leftmotor.setPower(1.0d);
        rightmotor.setPower(0d);
        sleep(825);
        telemetry.addData(">", "straight 3.5 sec");
        telemetry.update();
        leftmotor.setPower(1.0d);
        rightmotor.setPower(1.0d);
        sleep(3500);
        telemetry.addData(">", "right 90");
        telemetry.update();
        leftmotor.setPower(1.0);
        rightmotor.setPower(0d);
        sleep(825);
        telemetry.addData(">", "1 sec straight");
        telemetry.update();
        leftmotor.setPower(1.0d);
        rightmotor.setPower(1.0d);
        sleep(1000);
        telemetry.addData(">", "right 90");
        telemetry.update();
        leftmotor.setPower(1.0);
        rightmotor.setPower(0d);
        sleep(825);
        telemetry.addData(">", "1 sec straight");
        telemetry.update();
        leftmotor.setPower(1.0d);
        rightmotor.setPower(1.0d);
        sleep(1000);
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        telemetry.addData(">", "Ended");
        telemetry.update();
    }
}