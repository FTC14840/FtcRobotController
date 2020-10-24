package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.FierceFish;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "BraunTeleOpDcMotor")

@Disabled

public class BraunTeleOpDcMotor extends LinearOpMode {

    DcMotor testMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        testMotor = hardwareMap.dcMotor.get("testMotor");
        testMotor.setDirection(DcMotor.Direction.REVERSE);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while(opModeIsActive()){

            double motorPower = gamepad1.left_stick_y;

            if(gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < -0.1){
                testMotor.setPower(motorPower);
            } else {
                testMotor.setPower(0.0);
            }

        }

    }
}
