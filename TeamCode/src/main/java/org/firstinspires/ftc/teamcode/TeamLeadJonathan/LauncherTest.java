package org.firstinspires.ftc.teamcode.TeamLeadJonathan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

@TeleOp(name="Jonathan Launcher Test")

public class LauncherTest extends LinearOpMode {
    private DcMotor backTestLauncher;
    private DcMotor frontTestLauncher;


    @Override
    public void runOpMode() throws InterruptedException {

        backTestLauncher = opMode.hardwareMap.get(DcMotor.class, "backLauncher");
        frontTestLauncher = opMode.hardwareMap.get(DcMotor.class, "frontLauncher");
        backTestLauncher.setDirection(FORWARD);
        frontTestLauncher.setDirection(FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            backTestLauncher.setPower(1.0);
            frontTestLauncher.setPower(1.0);


        }
    }
}

