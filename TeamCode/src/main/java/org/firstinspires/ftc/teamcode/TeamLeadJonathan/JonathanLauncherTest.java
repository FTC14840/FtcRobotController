package org.firstinspires.ftc.teamcode.TeamLeadJonathan;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;


@TeleOp(name="Jonathan Launcher Test")

@Disabled

public class JonathanLauncherTest extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor backLauncher = null;
    private DcMotor frontLauncher = null;
    private static final double INCREMENT = 0.001;



    public void runOpMode() throws InterruptedException{
        telemetry.addData("Revving Up", "Prepare to Fire");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backLauncher = hardwareMap.get(DcMotor.class, "backLauncher");
        frontLauncher = hardwareMap.get(DcMotor.class, "frontLauncher");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery\

        waitForStart();
        backLauncher.setDirection(REVERSE);
        frontLauncher.setDirection(REVERSE);
        backLauncher.setPower(.80);
        frontLauncher.setPower(.80);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.log().clear();
            telemetry.addData("Encoder", "Launcher: %2d", frontLauncher.getCurrentPosition());
            telemetry.addData("Encoder", "Launcher: %2d", backLauncher.getCurrentPosition());
            telemetry.addData("Power", "Launcher: %.2f,", frontLauncher.getPower());
            telemetry.update();

            if (gamepad1.dpad_down) {
                backLauncher.setPower(backLauncher.getPower() - INCREMENT);
                frontLauncher.setPower(frontLauncher.getPower() - INCREMENT);
            }
            if (gamepad1.dpad_up) {
                backLauncher.setPower(backLauncher.getPower() + INCREMENT);
                frontLauncher.setPower(frontLauncher.getPower() + INCREMENT);
            }
        }
        backLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
