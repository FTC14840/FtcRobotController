package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class BraunVuforiaHardware {
    private LinearOpMode myOpMode;

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private double  driveAxial      = 0 ;   // Positive is forward
    private double  driveLateral    = 0 ;   // Positive is right
    private double  driveYaw        = 0 ;   // Positive is CCW

    public BraunVuforiaHardware() {
    }

    public void initDrive(LinearOpMode opMode) {

        myOpMode = opMode;

        frontLeft = myOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = myOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = myOpMode.hardwareMap.get(DcMotor.class, "frontLeft");

        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Positive input rotates counter clockwise
        frontRight.setDirection(DcMotor.Direction.FORWARD);// Positive input rotates counter clockwise
        backLeft.setDirection(DcMotor.Direction.REVERSE); // Positive input rotates counter clockwise
        backRight.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise

        //use RUN_USING_ENCODERS because encoders are installed.
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Stop all robot motion by setting each axis value to zero
        moveRobot(0,0,0) ;
    }

    public void manualDrive()  {
        // In this mode the Left stick moves the robot fwd & back, and Right & Left.
        // The Right stick rotates CCW and CW.

        //  (note: The joystick goes negative when pushed forwards, so negate it)
        setAxial(-myOpMode.gamepad1.left_stick_y);
        setLateral(myOpMode.gamepad1.left_stick_x);
        setYaw(-myOpMode.gamepad1.right_stick_x);
    }


    /***
     * void moveRobot(double axial, double lateral, double yaw)
     * Set speed levels to motors based on axes requests
     * @param axial     Speed in Fwd Direction
     * @param lateral   Speed in lateral direction (+ve to right)
     * @param yaw       Speed of Yaw rotation.  (+ve is CCW)
     */
    public void moveRobot(double axial, double lateral, double yaw) {
        setAxial(axial);
        setLateral(lateral);
        setYaw(yaw);
        moveRobot();
    }

    /***
     * void moveRobot()
     * This method will calculate the motor speeds required to move the robot according to the
     * speeds that are stored in the three Axis variables: driveAxial, driveLateral, driveYaw.
     * This code is setup for a three wheeled OMNI-drive but it could be modified for any sort of omni drive.
     *
     * The code assumes the following conventions.
     * 1) Positive speed on the Axial axis means move FORWARD.
     * 2) Positive speed on the Lateral axis means move RIGHT.
     * 3) Positive speed on the Yaw axis means rotate COUNTER CLOCKWISE.
     *
     * This convention should NOT be changed.  Any new drive system should be configured to react accordingly.
     */
    public void moveRobot() {
        // calculate required motor speeds to acheive axis motions
        double moveFrontLeft = driveYaw - driveAxial - (driveLateral * 0.5);
        double moveFrontRight = driveYaw - driveAxial - (driveLateral * 0.5);
        double moveBackLeft = driveYaw - driveAxial - (driveLateral * 0.5);
        double moveBackRight = driveYaw - driveAxial - (driveLateral * 0.5);

        // normalize all motor speeds so no values exceeds 100%.
        double max = Math.max(Math.abs(moveFrontLeft), Math.abs(moveFrontRight));
        max = Math.max(max, Math.abs(moveBackLeft));
        max = Math.max(max, Math.abs(moveBackRight));

        if (max > 1.0)
        {
            moveFrontLeft /= max;
            moveFrontRight /= max;
            moveBackLeft /= max;
            moveBackRight /= max;
        }

        // Set drive motor power levels.
        frontLeft.setPower(moveFrontLeft);
        frontRight.setPower(moveFrontRight);
        backLeft.setPower(moveBackLeft);
        backRight.setPower(moveBackRight);

        // Display Telemetry
        myOpMode.telemetry.addData("Axes  ", "A[%+5.2f], L[%+5.2f], Y[%+5.2f]", driveAxial, driveLateral, driveYaw);
        myOpMode.telemetry.addData("Wheels", "L[%+5.2f], R[%+5.2f], B[%+5.2f]", moveFrontLeft, moveFrontRight, moveBackLeft,moveBackRight);
    }


    public void setAxial(double axial)      {driveAxial = Range.clip(axial, -1, 1);}
    public void setLateral(double lateral)  {driveLateral = Range.clip(lateral, -1, 1); }
    public void setYaw(double yaw)          {driveYaw = Range.clip(yaw, -1, 1); }


    /***
     * void setMode(DcMotor.RunMode mode ) Set all drive motors to same mode.
     * @param mode    Desired Motor mode.
     */
    public void setMode(DcMotor.RunMode mode ) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }
}

