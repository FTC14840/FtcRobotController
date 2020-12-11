package org.firstinspires.ftc.teamcode.TeamLeadJonathan;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

// @Disabled
@TeleOp(name="Jonathan Velocity Test")

//total rpm for launcher configuration is 5,340rpms
public class JonathanVelocityLauncher extends LinearOpMode {

    DcMotorEx testLauncher;
//    private DcMotorEx leftLauncher;
//    private DcMotorEx rightLauncher;

    //Go builda 1620 RPM
//    double RPM = 1780;
//    double RPS = Math.abs(RPM / 60);
//    double TPS = Math.abs(RPS * 103.6);

    //Go builda 6000 RPM
    double RPM = 6000;
    double RPS = Math.abs(RPM / 60);
    double TPS = Math.abs(RPS * 28);


    double launcherIncrement = Math.abs(TPS * .01);

    public void runOpMode() throws InterruptedException {

        testLauncher = hardwareMap.get(DcMotorEx.class, "testLauncher");
        testLauncher.setDirection(REVERSE);
        testLauncher.setMode(STOP_AND_RESET_ENCODER);
        testLauncher.setMode(RUN_USING_ENCODER);
        testLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        leftLauncher = hardwareMap.get(DcMotorEx.class, "backLauncher");
//        rightLauncher = hardwareMap.get(DcMotorEx.class, "frontLauncher");
//        leftLauncher.setDirection(REVERSE);
//        rightLauncher.setDirection(REVERSE);
//        leftLauncher.setMode(STOP_AND_RESET_ENCODER);
//        rightLauncher.setMode(STOP_AND_RESET_ENCODER);
//        leftLauncher.setMode(RUN_USING_ENCODER);
//        rightLauncher.setMode(RUN_USING_ENCODER);
//        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        testLauncher.setPower(1);  // May not need
        testLauncher.setVelocity(TPS);
//        leftLauncher.setPower(1);
//        rightLauncher.setPower(1);
//        leftLauncher.setVelocity(TPS);
//        rightLauncher.setVelocity(TPS);


        while (opModeIsActive()) {
            if (gamepad1.x) {
                testLauncher.setVelocity(testLauncher.getVelocity() + launcherIncrement);
//                leftLauncher.setVelocity(leftLauncher.getVelocity() + launcherIncrement);
//                rightLauncher.setVelocity(rightLauncher.getVelocity() + launcherIncrement);
            }
            if (gamepad1.a) {
                testLauncher.setVelocity(testLauncher.getVelocity() - launcherIncrement);
//                leftLauncher.setVelocity(leftLauncher.getVelocity() - launcherIncrement);
//                rightLauncher.setVelocity(rightLauncher.getVelocity() - launcherIncrement);
            }
            if (testLauncher.getVelocity() > TPS) {
                testLauncher.setVelocity(TPS);
            }
            if (testLauncher.getVelocity() > TPS) {
                testLauncher.setVelocity(TPS);
            }
            if (testLauncher.getVelocity() < 0) {
                testLauncher.setVelocity(0);
            }
            if (testLauncher.getVelocity() < 0) {
                testLauncher.setVelocity(0);
            }
                telemetry.addData("Velocity", " %.2f,", testLauncher.getVelocity());
                telemetry.addData("RPMs", " %.2f,", testLauncher.getVelocity()/28*60);
//                telemetry.addData("BackPower", " %.2f,", leftLauncher.getVelocity()*3);
                telemetry.addData("Increment", " %.2f,", launcherIncrement);
                telemetry.update();
        }
    }
}