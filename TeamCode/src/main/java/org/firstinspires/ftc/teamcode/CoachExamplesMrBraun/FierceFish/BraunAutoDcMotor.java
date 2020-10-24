package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.FierceFish;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BraunAutoDcMotor")

@Disabled

public class BraunAutoDcMotor extends LinearOpMode {

    DcMotor testMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        testMotor = hardwareMap.dcMotor.get("testMotor");
        // or
        // testMotor = hardwareMap.get(DcMotor.class, "testmotor");
        testMotor.setDirection(DcMotor.Direction.REVERSE);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        testMotor.setPower(1.0);
        sleep(5000);
        testMotor.setPower(-0.5);
        sleep(2500);
        testMotor.setPower(0.0);

    }
}
