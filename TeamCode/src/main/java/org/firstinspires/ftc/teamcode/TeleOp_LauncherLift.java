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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.LAUNCHTEST;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.TROLLBOT;

/**
 * Created by FTC8424 on 9/15/2016.
 */

/**
 * Just a test for the launcher code with different speeds
 */

@TeleOp(name="TeleOp: LauncherLift", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class TeleOp_LauncherLift extends OpMode {



    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private HardwareHelper robot = new HardwareHelper(LAUNCHTEST);

    private double servoUpTime = 0;
    private double powerSetTime = 0;


    //private DcMotor leftMotorBack = null;
    //private DcMotor rightMotorFront = null;
    //private DcMotor rightMotorBack = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.robot_init(hardwareMap);
        telemetry.addData("Status", "Initialized");

       // launcher = hardwareMap.dcMotor.get("Launcher");

        //leftMotorBack  = hardwareMap.dcMotor.get("left back motor);
       // launch_lift = hardwareMap.servo.get("LaunchLift");
        //rightMotorBack  = hardwareMap.dcMotor.get("right back motor");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
         //launcher.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        // Set to FORWARD if using AndyMark motors
        //rightMotorBack.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //launch_lift.setPosition(0.9);



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




//        telemetry.addData("Path0", "Starting at %7d :%7d",
//                leftMotorFront.getCurrentPosition(),
//                rightMotorFront.getCurrentPosition());

       /* telemetry.addData("LaunchPower:  %.2f", launchPower);
        if (gamepad1.a && powerSetTime+2 < runtime.seconds()) {
            if ( launchPower - .1 < 0 ) {
                telemetry.addData("Launcher: %s", "At min speed");
            } else {
                launchPower -= .1;
                launcher.setPower(launchPower);
                powerSetTime = runtime.seconds();
            }
        }        */
        telemetry.addData("Servo", "LaunchServo set to " + robot.launchServo.getPosition());
        if (gamepad2.y && servoUpTime+2 < runtime.seconds()) {
            telemetry.addData("Status", "Debug 1 at: " + runtime.toString());
            if ( robot.launchServo.getPosition() == robot.launchliftStart ) {
                robot.launchServo.setPosition(robot.launchliftDeploy);
            }
            if (robot.launchServo.getPosition() == robot.launchliftDeploy && servoUpTime+1 > runtime.seconds());
                    robot.launchServo.setPosition(robot.launchliftStart);
            }



        servoUpTime = runtime.seconds();
            telemetry.addData("Status", "Debug 2 at: " + runtime.toString());
               /*launchPower += .1;
                launcher.setPower(launchPower);
                powerSetTime = runtime.seconds();
                */
            }
        //
        /*if (gamepad1.x) {
            launchPower = 0.0;
            launcher.setPower(launchPower);
        }
        if (gamepad1.b && servoUpTime+2 < runtime.seconds()) {
            if (launch_lift.getPosition() == 0) {
                launch_lift.setPosition(0.9);
            } else {
                launch_lift.setPosition(0);
            }
            servoUpTime = runtime.seconds();

        }

        //telemetry.addData("Servo", "Servo Set to " + launch_lift.getPosition());


//            launch_lift.setPosition(.25);
//            leftMotorFront.setTargetPosition(8000 + leftMotorFront.getCurrentPosition());
//            rightMotorFront.setTargetPosition(8000 + rightMotorFront.getCurrentPosition());
//            telemetry.addData("Path1", "Running to %7d :%7d",
//                    leftMotorFront.getTargetPosition(),
//                    rightMotorFront.getTargetPosition());
//            leftMotorFront.setPower(0.5);
//            rightMotorFront.setPower(0.5);
//
//            leftMotorFront.setTargetPosition(1976 + leftMotorFront.getCurrentPosition());
//            rightMotorFront.setTargetPosition(1976 + rightMotorFront.getCurrentPosition());
//            telemetry


        //}

    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()  {
        robot.normalDrive(0, 0);
    }
    }

