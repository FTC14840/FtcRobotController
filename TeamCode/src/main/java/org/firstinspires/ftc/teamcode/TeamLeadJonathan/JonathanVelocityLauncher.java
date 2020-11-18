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

@Disabled
@TeleOp(name="Jonathan Velocity Test")


//total rpm for launcher configuration is 5,340rpms
public class JonathanVelocityLauncher extends LinearOpMode {
    private DcMotorEx leftLauncher;
    private DcMotorEx rightLauncher;

    double RPM = 1780;
    double RPS = Math.abs(RPM / 60);
    double TPS = Math.abs(RPS * 103.6);
    double launcherIncrement = Math.abs(TPS * .01);

    public void runOpMode() throws InterruptedException {
        leftLauncher = hardwareMap.get(DcMotorEx.class, "backLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "frontLauncher");
        leftLauncher.setDirection(REVERSE);
        rightLauncher.setDirection(REVERSE);
        leftLauncher.setMode(STOP_AND_RESET_ENCODER);
        rightLauncher.setMode(STOP_AND_RESET_ENCODER);
        leftLauncher.setMode(RUN_USING_ENCODER);
        rightLauncher.setMode(RUN_USING_ENCODER);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        leftLauncher.setPower(1);
        rightLauncher.setPower(1);
        leftLauncher.setVelocity(TPS);
        rightLauncher.setVelocity(TPS);


        while (opModeIsActive()) {
            if (gamepad1.x) {
                leftLauncher.setVelocity(leftLauncher.getVelocity() + launcherIncrement);
                rightLauncher.setVelocity(rightLauncher.getVelocity() + launcherIncrement);
            }
            if (gamepad1.a) {
                leftLauncher.setVelocity(leftLauncher.getVelocity() - launcherIncrement);
                rightLauncher.setVelocity(rightLauncher.getVelocity() - launcherIncrement);
            }
            if (leftLauncher.getVelocity() > TPS) {
                leftLauncher.setVelocity(TPS);
            }
            if (rightLauncher.getVelocity() > TPS) {
                rightLauncher.setVelocity(TPS);
            }
            if (leftLauncher.getVelocity() < 0) {
                leftLauncher.setVelocity(0);
            }
            if (leftLauncher.getVelocity() < 0) {
                leftLauncher.setVelocity(0);
            }
                telemetry.addData("FrontPower", " %.2f,", rightLauncher.getVelocity()*3);
                telemetry.addData("BackPower", " %.2f,", leftLauncher.getVelocity()*3);
                telemetry.addData("LauncherPower", " %.2f,", launcherIncrement);
                telemetry.update();
        }
    }
}