package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Disabled

public class pidTest extends LinearOpMode {
    DcMotorEx motor;
    double targetVelocity = 900;
    double kP = 200.0;
    double kI = 20.0;
    double kD = 2.0;
    double F = 15.0;
    double currentVelocity;
    double maxVelocity = 0.0;
    double minVelocity = 2180; // Max velocity from Test

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "launcher");
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setVelocityPIDFCoefficients(kP,kI,kD,F);
        motor.setPositionPIDFCoefficients(5.0);

        waitForStart();

        motor.setVelocity(targetVelocity);

        while (opModeIsActive()) {

            currentVelocity = motor.getVelocity();
            telemetry.addData("current velocity", currentVelocity);
            telemetry.update();
        }
    }
}
