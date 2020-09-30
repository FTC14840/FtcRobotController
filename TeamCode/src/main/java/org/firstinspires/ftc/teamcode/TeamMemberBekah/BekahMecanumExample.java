package org.firstinspires.ftc.teamcode.TeamMemberBekah;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Bekah TeleOpMecanumExample")

@Disabled

public class BekahMecanumExample extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        double speed = 1;
        double direction = 1;
        boolean aButtonPad1 = false;
        boolean bButtonPad1 = false;
        double turnSensitivity = 2.0;

        waitForStart();

        while (opModeIsActive()) {

            double frontLeftClip;
            double frontRightClip;
            double backLeftClip;
            double backRightClip;

            frontLeft.setPower(frontLeftClip = speed * (direction * (( gamepad1.left_stick_y) + (gamepad1.left_stick_x * turnSensitivity))));
            frontRight.setPower(frontRightClip = speed * (direction * (( gamepad1.right_stick_y) + (gamepad1.right_stick_x * turnSensitivity))));
            backLeft.setPower(frontRightClip = speed * (direction * (( gamepad1.right_stick_y) + (gamepad1.right_stick_x * turnSensitivity))));











        }





    }

}
