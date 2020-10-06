package org.firstinspires.ftc.teamcode.TeamLeadJonathan;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;


@TeleOp(name="Launcher Test")

public class LauncherTest extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor backLauncher = null;
    private DcMotor frontLauncher = null;
    private static final double INCREMENT = 0.001;


    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backLauncher = hardwareMap.get(DcMotor.class, "backLauncher");
        frontLauncher = hardwareMap.get(DcMotor.class, "frontLauncher");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery\

        waitForStart();
        backLauncher.setDirection(FORWARD);
        frontLauncher.setDirection(FORWARD);
        backLauncher.setPower(1);
        frontLauncher.setPower(1);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

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