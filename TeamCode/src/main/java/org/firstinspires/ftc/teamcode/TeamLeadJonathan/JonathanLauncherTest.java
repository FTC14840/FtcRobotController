package org.firstinspires.ftc.teamcode.TeamLeadJonathan;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;


@TeleOp(name="Jonathan Launcher Test")


//total rpm for launcher configuration is 5,340rpms
public class JonathanLauncherTest extends LinearOpMode {

    private DcMotor leftLauncher;
    private DcMotor rightLauncher;

    double leftLauncherPower = 1.0;
    double rightLauncherPower = 1.0;

    private double INCREMENT2=.005;
    private double INCREMENT1=.001;


    public void runOpMode() throws InterruptedException{
        leftLauncher = hardwareMap.get(DcMotor.class, "backLauncher");
        rightLauncher = hardwareMap.get(DcMotor.class, "frontLauncher");
        telemetry.addData("Revving Up", "Prepare to Fire");
        telemetry.update();
        //Init specific launcher hardware for less confusion
        leftLauncher.setDirection(REVERSE);
        rightLauncher.setDirection(REVERSE);
        leftLauncher.setMode(STOP_AND_RESET_ENCODER);
        rightLauncher.setMode(STOP_AND_RESET_ENCODER);
        leftLauncher.setMode(RUN_USING_ENCODER);
        rightLauncher.setMode(RUN_USING_ENCODER);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLauncher.setPower(0.0);
        rightLauncher.setPower(0.0);


        waitForStart();
        //Sets motors to full power
        leftLauncher.setMode(STOP_AND_RESET_ENCODER);
        rightLauncher.setMode(STOP_AND_RESET_ENCODER);
        leftLauncher.setMode(RUN_WITHOUT_ENCODER);
        rightLauncher.setMode(RUN_WITHOUT_ENCODER);
//        leftLauncher.setMode(RUN_USING_ENCODER);
//        rightLauncher.setMode(RUN_USING_ENCODER);
        leftLauncher.setPower(leftLauncherPower);
        rightLauncher.setPower(rightLauncherPower);

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                leftLauncher.setMode(RUN_WITHOUT_ENCODER);
                rightLauncher.setMode(RUN_WITHOUT_ENCODER);
                //leftLauncherPower = leftLauncherPower + INCREMENT1;
                leftLauncherPower = Range.clip(leftLauncher.getPower() + INCREMENT1, 0, 1);
                leftLauncher.setPower(leftLauncherPower);
                //rightLauncherPower = rightLauncherPower + INCREMENT1;
                rightLauncherPower = Range.clip(rightLauncher.getPower() + INCREMENT1, 0, 1);
                rightLauncher.setPower(rightLauncherPower);
                leftLauncher.setMode(RUN_USING_ENCODER);
                rightLauncher.setMode(RUN_USING_ENCODER);

            }

            if (gamepad1.dpad_down) {
                leftLauncher.setMode(RUN_WITHOUT_ENCODER);
                rightLauncher.setMode(RUN_WITHOUT_ENCODER);
                //leftLauncherPower = leftLauncherPower - INCREMENT1;
                leftLauncherPower = Range.clip(leftLauncher.getPower() - INCREMENT1, 0, 1);
                leftLauncher.setPower(leftLauncherPower);
                //rightLauncherPower = rightLauncherPower - INCREMENT1;
                rightLauncherPower = Range.clip(rightLauncher.getPower() - INCREMENT1, 0, 1);
                rightLauncher.setPower(rightLauncherPower);
                leftLauncher.setMode(RUN_USING_ENCODER);
                rightLauncher.setMode(RUN_USING_ENCODER);
            }


//            if (gamepad1.dpad_down) {
//                rightLauncher.setPower(rightLauncher.getPower() - INCREMENT1);
//                leftLauncher.setPower(leftLauncher.getPower() - INCREMENT1);
//            }
//            if (gamepad1.dpad_up) {
//                rightLauncher.setPower(rightLauncher.getPower() + INCREMENT1);
//                leftLauncher.setPower(leftLauncher.getPower() + INCREMENT1);
//            }
//            if (gamepad1.dpad_left) {
//                rightLauncher.setPower(Range.clip(rightLauncher.getPower() - INCREMENT2, 0, 1));
//                leftLauncher.setPower(Range.clip(leftLauncher.getPower() - INCREMENT2, 0, 1));
//            }
//            if (gamepad1.dpad_right) {
//                rightLauncher.setPower(Range.clip(rightLauncher.getPower() + INCREMENT2, 0, 1));
//                leftLauncher.setPower(Range.clip(leftLauncher.getPower() + INCREMENT2, 0, 1));
//            }
//            if (gamepad1.a){
//                leftLauncher.setPower(1);
//                rightLauncher.setPower(1);
//            }
//            if (gamepad1.x){
//                leftLauncher.setPower(0.8);
//                rightLauncher.setPower(0.8);
//            }
//            if (gamepad1.y){
//                leftLauncher.setPower(0.6);
//                rightLauncher.setPower(0.6);
//            }
//            if (gamepad1.x){
//                leftLauncher.setPower(0.4);
//                rightLauncher.setPower(0.4);
//            }
            telemetry.log().clear();
            telemetry.addData("FrontPower", "Launcher: %.2f,", rightLauncher.getPower());
            telemetry.addData("BackPower", "Launcher: %.2f,", leftLauncher.getPower());
            telemetry.addData("FrontPower", "Launcher: %.2f,", rightLauncherPower);
            telemetry.addData("BackPower", "Launcher: %.2f,", leftLauncherPower);
            telemetry.update();
        }
    }
}

