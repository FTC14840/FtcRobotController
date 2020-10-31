package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.FierceFish;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "BraunAutoGyro")

@Disabled

public class BraunAutoGyro extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    BNO055IMU imu;  // Inertial Measurement Unit - I2C Bus Zero

    private static final double TICKS = 537.6; // goBulda 312's = 537.6, NeveRest/AndyMark = 1120, Tetrix = 1440
    private static final double GEARRATIO = 1.0;
    private static final double WHEELDIAMETERINCHES = 4.0;
    private static final double TICKSTOINCHES = (TICKS * GEARRATIO) / (Math.PI * WHEELDIAMETERINCHES);

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        /**
         * Understanding the IMU axes.
         * Start with the Control Hub flat parallel to the floor with you reading the label from the back of the robot.
         * West is -x and East is +x
         * North is +y and South is -y
         * Up toward you is +z while -z goes through the floor.
         * Right hand rule - We are reading the z-axis with left being (+) and right being (-).
         */

        gyroForward(12,.50,0,1000);
        gyroReverse(12,.50,0,1000);
        gyroStrafeLeft(12,.50,0,1000);
        gyroStrafeRight(12, .50,0,1000);
        gyroLeft(.50,90,1000);
        gyroRight(.50, -90,1000);
        gyroLeft(.50, 0,1000);

    }

    /**
     * Automomous Methods
     */

    public void gyroForward(int distance, double power, double angle, int pause) throws InterruptedException {
        if (opModeIsActive()) {
            stopAndResetEncoder();
            driveByInches(distance);
            runToPosition();
            while (opModeIsActive()) {
                double gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double errorMultiple = 1.0;
                double error = (errorMultiple * (gyroHeading - angle) / 100);
                if (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    frontLeft.setPower(power + error);
                    frontRight.setPower(power - error);
                    backLeft.setPower(power + error);
                    backRight.setPower(power - error);
                } else {
                    stopDriving();
                    sleep(pause);
                    break;
                }
            }
        }
    }

    public void gyroReverse(int distance, double power, double angle, int pause) throws InterruptedException {
        if (opModeIsActive()) {
            stopAndResetEncoder();
            driveByInches(-distance);
            runToPosition();
            while (opModeIsActive()) {
                double gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double errorMultiple = 1.0;
                double error = (errorMultiple * (gyroHeading - angle) / 100);
                if (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    frontLeft.setPower(power - error);
                    frontRight.setPower(power + error);
                    backLeft.setPower(power - error);
                    backRight.setPower(power + error);
                } else {
                    stopDriving();
                    sleep(pause);
                    break;
                }
            }
        }
    }

    public void gyroStrafeLeft(int distance, double power, double angle, int pause) throws InterruptedException {
        if (opModeIsActive()) {
            stopAndResetEncoder();
            strafeByInches(-distance);
            runToPosition();
            while (opModeIsActive()) {
                double gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double errorMultiple = 1.0;
                double error = (errorMultiple * (gyroHeading - angle) / 100);
                if (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    frontLeft.setPower(power + error);
                    frontRight.setPower(power + error);
                    backLeft.setPower(power - error);
                    backRight.setPower(power - error);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void gyroStrafeRight(int distance, double power, double angle, int pause) throws InterruptedException {
        if (opModeIsActive()) {
            stopAndResetEncoder();
            strafeByInches(distance);
            runToPosition();
            while (opModeIsActive()) {
                double gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double errorMultiple = 1.0;
                double error = (errorMultiple * (gyroHeading - angle) / 100);
                if (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    frontLeft.setPower(power - error);
                    frontRight.setPower(power - error);
                    backLeft.setPower(power + error);
                    backRight.setPower(power + error);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void gyroLeft(double power, double angle, int pause) throws InterruptedException {
        if (opModeIsActive()) {
            double halfPower = power/2;
            double quarterPower = power/4;
            int cyckeTime = 100;
            stopAndResetEncoder();
            runUsingEncoder();
            while (opModeIsActive()) {
                double gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (gyroHeading < angle - 60) {
                    frontLeft.setPower(power);
                    frontRight.setPower(-power);
                    backLeft.setPower(power);
                    backRight.setPower(-power);
                    sleep(cyckeTime);
                } else if(gyroHeading < angle - 40) {
                    frontLeft.setPower(halfPower);
                    frontRight.setPower(-halfPower);
                    backLeft.setPower(halfPower);
                    backRight.setPower(-halfPower);
                    sleep(cyckeTime);
                } else if(gyroHeading < angle - 20) {
                    frontLeft.setPower(quarterPower);
                    frontRight.setPower(-quarterPower);
                    backLeft.setPower(quarterPower);
                    backRight.setPower(-quarterPower);
                    sleep(cyckeTime * 2);
                } else if(gyroHeading > angle) {
                    frontLeft.setPower(-quarterPower);
                    frontRight.setPower(quarterPower);
                    backLeft.setPower(-quarterPower);
                    backRight.setPower(quarterPower);
                    sleep(cyckeTime * 2);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void gyroRight(double power, double angle, int pause) throws InterruptedException {
        if (opModeIsActive()) {
            double halfPower = power/2;
            double quarterPower = power/4;
            int cyckeTime = 100;
            stopAndResetEncoder();
            runUsingEncoder();
            while (opModeIsActive()) {
                double gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (gyroHeading > angle + 60) {
                    frontLeft.setPower(-power);
                    frontRight.setPower(power);
                    backLeft.setPower(-power);
                    backRight.setPower(power);
                    sleep(cyckeTime);
                } else if (gyroHeading > angle + 40) {
                    frontLeft.setPower(-halfPower);
                    frontRight.setPower(halfPower);
                    backLeft.setPower(-halfPower);
                    backRight.setPower(halfPower);
                    sleep(cyckeTime);
                } else if (gyroHeading > angle + 20 ) {
                    frontLeft.setPower(-quarterPower);
                    frontRight.setPower(quarterPower);
                    backLeft.setPower(-quarterPower);
                    backRight.setPower(quarterPower);
                    sleep(cyckeTime * 2);
                } else if (gyroHeading < angle) {
                    frontLeft.setPower(quarterPower);
                    frontRight.setPower(-quarterPower);
                    backLeft.setPower(quarterPower);
                    backRight.setPower(-quarterPower);
                    sleep(cyckeTime * 2);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    /**
     *  Drive Train Methods
     */

    public void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void stopAndResetEncoder() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEncoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void driveByInches(int distance) {
        frontLeft.setTargetPosition(-distance * (int) TICKSTOINCHES);
        frontRight.setTargetPosition(-distance * (int) TICKSTOINCHES);
        backLeft.setTargetPosition(-distance * (int) TICKSTOINCHES);
        backRight.setTargetPosition(-distance * (int) TICKSTOINCHES);
    }

    public void strafeByInches(int distance) {
        frontLeft.setTargetPosition(distance * (int) TICKSTOINCHES);
        frontRight.setTargetPosition(-distance * (int) TICKSTOINCHES);
        backLeft.setTargetPosition(-distance * (int) TICKSTOINCHES);
        backRight.setTargetPosition(distance * (int) TICKSTOINCHES);
    }

}
