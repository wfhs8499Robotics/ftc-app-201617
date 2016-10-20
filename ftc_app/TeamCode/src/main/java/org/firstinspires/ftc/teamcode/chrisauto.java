import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Ftc Test autonomous", group = "Autonomous OpMode")
//@Override
public class chrisauto extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        DcMotor leftmotor = null;
        DcMotor rightmotor = null;
        leftmotor = hardwareMap.dcMotor.get("left motor");
        leftmotor.setDirection(DcMotor.Direction.REVERSE);
        rightmotor = hardwareMap.dcMotor.get("right motor");

        telemetry.addData(">", "Finished initiating, press play to start moving");
        telemetry.update();

        waitForStart();
        telemetry.addData(">", "Started");
        telemetry.update();
        leftmotor.setPower(1.0d);
        rightmotor.setPower(1.0d);
        sleep(2000);
        telemetry.addData(">", "right");
        telemetry.update();
        leftmotor.setPower(1.0d);
        rightmotor.setPower(0);
        sleep(1000);
        telemetry.addData(">", "short");
        telemetry.update();
        leftmotor.setPower(1.0d);
        rightmotor.setPower(1.0d);
        sleep(1000);
        telemetry.addData(">", "left");
        telemetry.update();
        leftmotor.setPower(0);
        rightmotor.setPower(1.0d);
        sleep(1000);
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        telemetry.addData(">", "Ended");
        telemetry.update();

    }
}