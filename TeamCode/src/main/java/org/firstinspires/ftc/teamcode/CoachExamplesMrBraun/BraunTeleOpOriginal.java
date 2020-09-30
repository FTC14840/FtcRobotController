package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

// @Disabled

//Changes

// Register class as TeleOp and give it a name
@TeleOp(name ="ExampleTeleOpMecanum")

@Disabled

// Define class and extend LinerOpMode
public class BraunTeleOpOriginal extends LinearOpMode {

    // define global variables
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Override RunOpMode method from LinearOPMode class
    @Override
    public void runOpMode() {

        // Map global variables to config file on Robot Controller
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        // Reverse right side motors so they drive forward
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // define local variables for driver control
        double speed = 1;
        double direction = 1;
        boolean aButtonPad1 = false;
        boolean bButtonPad1 = false;
        double turnMultiple = 2.0;

        // Wait for driver to press play
        waitForStart();

        // Until driver presses Stop, run this code
        while (opModeIsActive()) {

            // Define clip variables for motor power
            double frontLeftClip;
            double frontRightClip;
            double backLeftClip;
            double backRightClip;

            // Place the robot on blocks a correct these vectors.  They may be different based on gear ratios and motor orientation.
            /** Forward & Reverse Vectors
            frontLeft.setPower(frontLeftClip = (gamepad1.left_stick_y * speed * direction));
            frontRight.setPower(frontRightClip = (gamepad1.left_stick_y * speed * direction));
            backLeft.setPower(backLeftClip = (gamepad1.left_stick_y * speed * direction));
            backRight.setPower(backRightClip = (gamepad1.left_stick_y * speed * direction));

            *** Strafe Vectors
            frontLeft.setPower(frontLeftClip = (gamepad1.left_stick_x * speed * direction));
            frontRight.setPower(frontRightClip = -(gamepad1.left_stick_x * speed * direction));
            backLeft.setPower(backLeftClip = -(gamepad1.left_stick_x * speed * direction));
            backRight.setPower(backRightClip = (gamepad1.left_stick_x * speed * direction));

            *** Turning Vectors
            frontLeft.setPower(frontLeftClip = -(gamepad1.right_stick_x * speed * turnMultiple));
            frontRight.setPower(frontRightClip = (gamepad1.right_stick_x * speed * turnMultiple));
            backLeft.setPower(backLeftClip = -(gamepad1.right_stick_x * speed * turnMultiple));
            backRight.setPower(backRightClip = (gamepad1.right_stick_x * speed * turnMultiple));

            *** Vector Addition ( from Physics) combining all three power settings for each motor above
            frontLeft.setPower(frontLeftClip = (gamepad1.left_stick_y * speed * direction) + (gamepad1.left_stick_x * speed * direction) - (gamepad1.right_stick_x * speed * turnMultiple));
            frontRight.setPower(frontRightClip = (gamepad1.left_stick_y * speed * direction) - (gamepad1.left_stick_x * speed * direction) + (gamepad1.right_stick_x * speed * turnMultiple));
            backLeft.setPower(backLeftClip = (gamepad1.left_stick_y * speed * direction) - (gamepad1.left_stick_x * speed * direction) - (gamepad1.right_stick_x * speed * turnMultiple));
            backRight.setPower(backRightClip = (gamepad1.left_stick_y * speed * direction) + (gamepad1.left_stick_x * speed * direction) + (gamepad1.right_stick_x * speed * turnMultiple));

            *** Factor Speed from each line using Algebra
            frontLeft.setPower(frontLeftClip = speed * ((gamepad1.left_stick_y * direction) + (gamepad1.left_stick_x * direction) - (gamepad1.right_stick_x * turnMultiple)));
            frontRight.setPower(frontRightClip = speed * ((gamepad1.left_stick_y * direction) - (gamepad1.left_stick_x * direction) + (gamepad1.right_stick_x * turnMultiple)));
            backLeft.setPower(backLeftClip = speed * ((gamepad1.left_stick_y * direction) - (gamepad1.left_stick_x * direction) - (gamepad1.right_stick_x * turnMultiple)));
            backRight.setPower(backRightClip = speed * ((gamepad1.left_stick_y * direction) + (gamepad1.left_stick_x * direction) + (gamepad1.right_stick_x * turnMultiple)));
            **/

            // Final, after Factor Direction
            frontLeft.setPower(frontLeftClip = speed * (direction * ((gamepad1.left_stick_y) + (gamepad1.left_stick_x)) - (gamepad1.right_stick_x * turnMultiple)));
            frontRight.setPower(frontRightClip = speed * (direction * ((gamepad1.left_stick_y) - (gamepad1.left_stick_x)) + (gamepad1.right_stick_x * turnMultiple)));
            backLeft.setPower(backLeftClip = speed * (direction * ((gamepad1.left_stick_y) - (gamepad1.left_stick_x)) - (gamepad1.right_stick_x * turnMultiple)));
            backRight.setPower(backRightClip = speed * (direction * ((gamepad1.left_stick_y) + (gamepad1.left_stick_x)) + (gamepad1.right_stick_x * turnMultiple)));

            // Clipping (limiting) power values to -1.0 to 1.0
            frontLeft.setPower(Range.clip(frontLeftClip, -1.0, 1.0));
            frontRight.setPower(Range.clip(frontRightClip, -1.0, 1.0));
            backLeft.setPower(Range.clip(backLeftClip, -1.0, 1.0));
            backRight.setPower(Range.clip(backRightClip, -1.0, 1.0));

            // else if logic for speed control on button A
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

            // else if logic for direction control on button B
            if (gamepad1.b) {
                bButtonPad1 = true;
            } else if (bButtonPad1) {
                bButtonPad1 = false;
                direction = -direction;
            }
        }
    }
}