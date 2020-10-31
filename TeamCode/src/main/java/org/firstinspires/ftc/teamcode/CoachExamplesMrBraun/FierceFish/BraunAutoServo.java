package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.FierceFish;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BraunAutoServo")

@Disabled

public class BraunAutoServo extends LinearOpMode {

    Servo testServo;
    CRServo testCrServo;

    @Override
    public void runOpMode() throws InterruptedException {

        testServo = hardwareMap.servo.get("testServo");
        testServo.setDirection(Servo.Direction.REVERSE);

        testCrServo = hardwareMap.crservo.get("testCrServo");
        testCrServo.setDirection((DcMotorSimple.Direction.REVERSE));

        waitForStart();

        testServo.setPosition(0.0);  // between 0 and 1
        sleep(1000);

        // Discuss for loops to get servos to slow down in their movement

        testCrServo.setPower(-1.0);  // between -1.0 and 1.0




    }
}
