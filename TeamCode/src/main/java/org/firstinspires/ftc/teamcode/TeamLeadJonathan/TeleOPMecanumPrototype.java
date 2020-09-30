// FTC package name
package org.firstinspires.ftc.teamcode.TeamLeadJonathan;

// Auto Imports

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

// @Disabled

// Name under TeleOp List
@TeleOp(name ="This Is A Serious Name")

// Begin of LinearOpMode Class
public class TeleOPMecanumPrototype extends LinearOpMode {

    // Defining or global variables for Java
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() {

        // Maping our Java variables to the config file
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        // Reverse right motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        double speed = 1;
        double direction = 1;
        boolean aButtonPad1 = false;
        boolean bButtonPad1 = false;
        double turnMultiple = 1.0;

        waitForStart();

        while (opModeIsActive()) {

            double frontLeftClip;
            double frontRightClip;
            double backLeftClip;
            double backRightClip;

// Forward & Reverse Vectors
//            frontLeft.setPower(frontLeftClip = (gamepad1.left_stick_y * speed * direction));
//            frontRight.setPower(frontRightClip = (gamepad1.left_stick_y * speed * direction));
//            backLeft.setPower(backLeftClip = (gamepad1.left_stick_y * speed * direction));
//            backRight.setPower(backRightClip = (gamepad1.left_stick_y * speed * direction));

// Turning Vectors
//            frontLeft.setPower(frontLeftClip = -(gamepad1.right_stick_x * speed * direction * turnMultiple));
//            frontRight.setPower(frontRightClip = (gamepad1.right_stick_x * speed * direction * turnMultiple));
//            backLeft.setPower(backLeftClip = -(gamepad1.right_stick_x * speed * direction * turnMultiple));
//            backRight.setPower(backRightClip = (gamepad1.right_stick_x * speed * direction * turnMultiple));

// Strafe Vectors
//            frontLeft.setPower(frontLeftClip = (gamepad1.left_stick_x * speed));
//            frontRight.setPower(frontRightClip = -(gamepad1.left_stick_x * speed));
//            backLeft.setPower(backLeftClip = -(gamepad1.left_stick_x * speed));
//            backRight.setPower(backRightClip = (gamepad1.left_stick_x * speed));


//            frontLeft.setPower(frontLeftClip = (gamepad1.left_stick_y * speed * direction)-(gamepad1.right_stick_x * speed)+(gamepad1.left_stick_x * speed * direction));
//            frontRight.setPower(frontRightClip = (gamepad1.left_stick_y * speed * direction)+(gamepad1.right_stick_x * speed)-(gamepad1.left_stick_x * speed*direction));
//            backLeft.setPower(backLeftClip = (gamepad1.left_stick_y * speed * direction)-(gamepad1.right_stick_x * speed)-(gamepad1.left_stick_x * speed*direction));
//            backRight.setPower(backRightClip = (gamepad1.left_stick_y * speed * direction)+(gamepad1.right_stick_x * speed)+(gamepad1.left_stick_x * speed*direction));

            frontLeft.setPower(frontLeftClip=(speed*(direction*(gamepad1.left_stick_y))+(gamepad1.left_stick_x)-(gamepad1.right_stick_x*turnMultiple)));
            frontRight.setPower(frontRightClip=(speed*(direction*(gamepad1.left_stick_y))-(gamepad1.left_stick_x)+(gamepad1.right_stick_x*turnMultiple)));
            backLeft.setPower(backLeftClip=(speed*(direction*(gamepad1.left_stick_y))-(gamepad1.left_stick_x)-(gamepad1.right_stick_x*turnMultiple)));
            backRight.setPower(backRightClip=(speed*(direction*(gamepad1.left_stick_y))+(gamepad1.left_stick_x)+(gamepad1.right_stick_x*turnMultiple)));

            frontLeft.setPower(Range.clip(frontLeftClip, -1.0, 1.0));
            frontRight.setPower(Range.clip(frontRightClip, -1.0, 1.0));
            backLeft.setPower(Range.clip(backLeftClip, -1.0, 1.0));
            backRight.setPower(Range.clip(backRightClip, -1.0, 1.0));

            if (gamepad1.a) {
                aButtonPad1 = true;
            } else if (aButtonPad1) {
                aButtonPad1 = false;
                if (speed == 0.75) {
                    speed = 1;
                } else {
                    speed = 0.75;
                }
            }

            if (gamepad1.b) {
                bButtonPad1 = true;
            } else if (bButtonPad1) {
                bButtonPad1 = false;
                direction = -direction;
            }

        }
    }
}