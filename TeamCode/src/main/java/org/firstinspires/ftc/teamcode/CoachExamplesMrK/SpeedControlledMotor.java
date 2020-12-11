/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.CoachExamplesMrK;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static java.lang.Math.abs;

/**
 * This OpMode is attempting to get a motor to run to a set velocity.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "Motor".
 * 11/9/20 notes - up to now, the only time SetVelocity seems to work is when is it set to zero, and that has only been tested as the initial speed.
 * The problem appears to have been that the motor was wired in reverse (red to black, and black to red) which is okay accept that it apparently
 * messes up the encoder.  I had also tried "initializing" the motor by setting power to zero before the loop, but that turned out not to have any
 * impact.
 *
 * Experimented with with maximum speed for this motor type.  It appears to be a little less than 3000.  This corresponds well to the Andy Mark documentation
 * which says 1120 ticks per revolution for this motor.
 *
 * Still have a problem with displaying the correct velocity.  Moving getVelocity out of the telemetry statement did not help.  Delaying one second  after setting the velocity
 * before getting the velocity did not help.  Delaying three seconds afteer using SetVelociy helped, but it is stlll not perfect.  There are some issues with the program.
 *
 *
 * */
@TeleOp(name = "Speed Controlled Motor", group = "Concept")
//@Disabled
public class SpeedControlledMotor extends LinearOpMode {

    // Define class members
    DcMotorEx motor;
    double motorVelocity = 3000;
    double zeroVelocity = 0;
    double actualVel = 0;
    int mode = 1;
    int count = 0;
    //String mode;

    @Override
    public void runOpMode() {

        // Connect to motor
        // Change the text in quotes to match any motor name on your robot.
        motor = hardwareMap.get(DcMotorEx.class, "Motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//not sure this has any impact
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the start button and display some messages in the meantime
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.addData("Current Run Mode",motor.getMode());
        telemetry.update();

        waitForStart();

//Initialize the motor to zero power.  This had no immpact.
        //motor.setPower(0);

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {
            if(mode == 0) {
// This will turn the motor per the value of the motor Velocity variable.
// Also, the value of the target velocity and the actual velocity will be displayed
// After three seconds, the target speed will change.
                //Start with the target velocity at 0.
                motor.setVelocity(zeroVelocity);
                sleep(1000);
                actualVel = motor.getVelocity();
                telemetry.addData("Target Motor Velocity", "%5.2f", zeroVelocity);
                telemetry.addData("Actual Motor Velocity", "%5.2f", actualVel);
                telemetry.update();
                sleep(2000);
                telemetry.clear();

                //Set target velocity to current value of motorVelocity
                motor.setVelocity(motorVelocity);
                sleep(1000);
                actualVel = motor.getVelocity();
                telemetry.addData("Target Motor Velocity", "%5.2f", motorVelocity);
                telemetry.addData("Actual Motor Velocity", "%5.2f", actualVel);
                telemetry.update();
                sleep(2000);
                telemetry.clear();

                //Flip the sign of target velocity
/*            motorVelocity = -motorVelocity;
            motor.setVelocity(motorVelocity);
            sleep(1000);
            actualVel = motor.getVelocity();
            telemetry.addData("Target Motor Velocity", "%5.2f", motorVelocity);
            telemetry.addData("Actual Motor Velocity", "%5.2f", actualVel);
            telemetry.update();
            sleep(2000);
            telemetry.clear();
*/
                //Reduce the target velocity by 200.
                motorVelocity = abs(motorVelocity) - 200;

            }

            if(mode == 1) {
                motorVelocity = 2600;
                motor.setVelocity(motorVelocity);
                while (count < 10) {
                    actualVel = motor.getVelocity();
                    telemetry.addData("Target Motor Velocity", "%5.2f", motorVelocity);
                    telemetry.addData("Actual Motor Velocity", "%5.2f", actualVel);
                    telemetry.addData("Count =", count);
                    telemetry.update();
                    sleep(1000);
                    telemetry.clear();
                    count = count+1;
                }
            }
            //From here the program will iterate back to the While(opModeisActive) statement and
            // set speed to zero, then the opposite of that last velocity, the 90% of the velocity.


        }

        // Turn off motor and signal done;

        motor.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
