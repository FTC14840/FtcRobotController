package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.FierceFish;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "BraunTeleOpDcMotor")

@Disabled

public class BraunTeleOpDcMotor extends LinearOpMode {

    DcMotor firstTestMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        firstTestMotor = hardwareMap.get(DcMotor.class, "testMotor");
        firstTestMotor.setDirection(DcMotor.Direction.REVERSE);
        firstTestMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {

            firstTestMotor.setPower(gamepad1.left_stick_y);

            // or

            double firstTestMotorPower = gamepad1.left_stick_y;

            if (firstTestMotorPower > 0.1 || firstTestMotorPower < -0.1) {
                firstTestMotor.setPower(firstTestMotorPower);
            }
        }
    }
}
