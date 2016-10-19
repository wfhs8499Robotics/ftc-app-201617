import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Ftc Test autonomous", group = "Autonomous OpMode")
//@Override
public class chrisauto extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        DcMotor leftmotor = null;
        DcMotor rightmotor = null;
        leftmotor = hardwareMap.dcMotor.get("left motor");
        rightmotor = hardwareMap.dcMotor.get("right motor");

        waitForStart();

        leftmotor.setPower(1.0d);
        rightmotor.setPower(1.0d);
        sleep(2000);
        leftmotor.setPower(0);
        rightmotor.setPower(0);

    }
}