package org.firstinspires.ftc.teamcode.TeamLeadJonathan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.Serializable;

@TeleOp(name="JonathanIntakeTest")
public class JonathanIntakeTest extends LinearOpMode {
    private Servo intakeServo;
    private DcMotor intakeMotor;
    private double INCREMENT=.001;

    @Override
    public void runOpMode() throws InterruptedException{
        intakeServo=hardwareMap.get(Servo.class,"intakeServo");
        intakeMotor=hardwareMap.get(DcMotor.class,"intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeServo.setPosition(0);
        waitForStart();
        intakeMotor.setPower(1);
        while (opModeIsActive()){
            if (gamepad2.dpad_up){
                intakeMotor.setPower(intakeMotor.getPower()+INCREMENT);
            }
            if (gamepad2.dpad_down){
                intakeMotor.setPower(intakeMotor.getPower()-INCREMENT);
            }

        }

    }

}

