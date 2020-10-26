package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name="Braun Launcher")

@Disabled

public class BraunLauncher extends LinearOpMode {

    private DcMotor backLauncher = null;
    private DcMotor frontLauncher = null;
    private static final double INCREMENT = 0.001;

    public void runOpMode() {

        telemetry.log().clear();
        telemetry.addData("Spinning Up", "Prepare to Launch");
        telemetry.update();

        backLauncher = hardwareMap.get(DcMotor.class, "backLauncher");
        frontLauncher = hardwareMap.get(DcMotor.class, "frontLauncher");

        backLauncher.setDirection(REVERSE);
        frontLauncher.setDirection(REVERSE);

        backLauncher.setPower(.60);
        frontLauncher.setPower(.60);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                backLauncher.setPower(backLauncher.getPower() + INCREMENT);
                frontLauncher.setPower(frontLauncher.getPower() + INCREMENT);
            }

            if (gamepad1.dpad_down) {
                backLauncher.setPower(backLauncher.getPower() - INCREMENT);
                frontLauncher.setPower(frontLauncher.getPower() - INCREMENT);
            }

            telemetry.log().clear();
            telemetry.addData("Launcher", "Front: %.2f, Back: %.2f", frontLauncher.getPower(), backLauncher.getPower());
            telemetry.update();

        }

        backLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
}
