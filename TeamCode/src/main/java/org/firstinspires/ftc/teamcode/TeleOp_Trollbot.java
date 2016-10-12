/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import java.util.concurrent.TimeUnit;

/**
 * Created by FTC8424 on 9/15/2016.
 */

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="TeleOp: Trollbot", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class TeleOp_Trollbot extends OpMode {



    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotorFront = null;
    //private DcMotor leftMotorBack = null;
    private DcMotor rightMotorFront = null;
    //private DcMotor rightMotorBack = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotorFront = hardwareMap.dcMotor.get("L Front");
        //leftMotorBack  = hardwareMap.dcMotor.get("left back motor);
        rightMotorFront = hardwareMap.dcMotor.get("R Front");
        //rightMotorBack  = hardwareMap.dcMotor.get("right back motor");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
         rightMotorFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        // Set to FORWARD if using AndyMark motors
        //rightMotorBack.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        telemetry.addData("Status", "Running: " + runtime.toString());

        /* This code will take the values from the gamepad, negate them,
        and then square that value. This will make the values sent to the
        robot more sensitive, giving the driver more control / precision.
         */
        /*double rightStickVal = -gamepad1.right_stick_y;
        double leftStickVal = -gamepad1.left_stick_y;
        double rightSquaredVal = rightStickVal * rightStickVal;
        double leftSquaredVal = leftStickVal * leftStickVal;

        //rightMotorFront.setPower(rightSquaredVal);
        //leftMotorFront.setPower(leftSquaredVal);

        if (rightStickVal < 0) {

            rightMotorFront.setPower(-rightSquaredVal);
        } else {
            rightMotorFront.setPower(rightSquaredVal);
        }
        if (leftStickVal < 0) {
            leftMotorFront.setPower(-leftSquaredVal);
        } else {
            leftMotorFront.setPower(leftSquaredVal);
        }
        */




        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftMotorFront.getCurrentPosition(),
                rightMotorFront.getCurrentPosition());

        if (gamepad1.a) {
            leftMotorFront.setTargetPosition(8000 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(8000 + rightMotorFront.getCurrentPosition());
            telemetry.addData("Path1", "Running to %7d :%7d",
                    leftMotorFront.getTargetPosition(),
                    rightMotorFront.getTargetPosition());
            leftMotorFront.setPower(0.5);
            rightMotorFront.setPower(0.5);

            //Turning to align at first beacon
            leftMotorFront.setPower(-0.5);
            rightMotorFront.setPower(0.5);

            //Backing up at first beacon
            leftMotorFront.setTargetPosition(728 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(728 + rightMotorFront.getCurrentPosition());

            //Turning right to go towards second beacon. How to wait??
            leftMotorFront.setPower(0.5);
            rightMotorFront.setPower(-0.5);

            //TimeUnit.MILLISECONDS.sleep(1000);



            leftMotorFront.setTargetPosition(1976 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(1976 + rightMotorFront.getCurrentPosition());


        }

    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}