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
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLTELEOP;

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
@TeleOp(name="TeleOp: FullRobot FullSpeed", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class FullRobotFullSpeed extends OpMode {


    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private HardwareHelper robot = new HardwareHelper(FULLTELEOP);

//    private Servo LPush;
//    double LPushStart = 0.1;
//    double LPushEnd = 0.9;
//    double LPushPower = 1.1;

    private double LservoUpTime = 0;
    private double RservoUpTime = 0;
    private double powerSetTime = 0;
    private double servoUpTime = 0;
    double launchPress = 0;
    double lastStateChange = 0;
    int launcherState = 0;
    double SpeedChange = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.robot_init(hardwareMap);
        telemetry.addData("Status", "Initialized");
//        LPush = hardwareMap.servo.get("left_push");
//        LPush.setPosition(LPushStart);
//        wait(2000);
//        LPush.setPosition(LPushEnd);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */


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

        double rightStickVal = -gamepad1.right_stick_y;
        double leftStickVal = -gamepad1.left_stick_y;
        double rightSquaredVal = rightStickVal * rightStickVal;
        double leftSquaredVal = leftStickVal * leftStickVal;

        if (rightStickVal < 0) rightSquaredVal = -rightSquaredVal;
        if (leftStickVal < 0) leftSquaredVal = -leftSquaredVal;
        telemetry.addData("NormalDrive:", "Lft Power %.2f, Rgt Power %.2f", leftSquaredVal, rightSquaredVal);
        robot.normalDrive(this, leftSquaredVal, rightSquaredVal);

        telemetry.addData("Path0", "Position at %7d :%7d",
                robot.leftBackDrive.getCurrentPosition(),
                robot.rightBackDrive.getCurrentPosition());

        telemetry.addData("Status", "Debug 1 at: " + runtime.toString());

        if (gamepad2.left_bumper && (LservoUpTime + 2) < runtime.seconds()) {
            //robot.lpushStart robot.lpushDeploy
            if (robot.leftPush.getPosition() == robot.lpushStart) {
                robot.leftPush.setPosition(robot.lpushDeploy);

            } else {
                robot.leftPush.setPosition(robot.lpushStart);
            }
            LservoUpTime = runtime.seconds();
        }
        telemetry.addData("Status", "Debug 2 at: " + runtime.toString());
        telemetry.addData("Servo", " 1 leftPush Servo Set to " + robot.leftPush.getPosition());

        if (gamepad2.right_bumper && (RservoUpTime + 2) < runtime.seconds()) {
            telemetry.addData("Status", "Debug 3 at: " + runtime.toString());
            // robot.rpushStart robot.rpushDeploy
            if (robot.rightPush.getPosition() == robot.rpushStart) {
                robot.rightPush.setPosition(robot.rpushDeploy);
            } else {
                robot.rightPush.setPosition(robot.rpushStart);
            }
            RservoUpTime = runtime.seconds();
        }

        telemetry.addData("Status", "Debug 4 at: " + runtime.toString());
        telemetry.addData("Servo", "2 rightPush Servo Set to " + robot.rightPush.getPosition());
        telemetry.addData("Servo", "LaunchServo set to " + robot.launchServo.getPosition());


        if (gamepad2.y && (servoUpTime+2) < runtime.seconds()) {
            telemetry.addData("Status", "Debug 1 at: " + runtime.toString());
            if (robot.launchServo.getPosition() == robot.launchliftStart) {
                robot.launchServo.setPosition(robot.launchliftDeploy);


            }

            servoUpTime = runtime.seconds();
        }
             if (robot.launchServo.getPosition() == robot.launchliftDeploy && servoUpTime+1 < runtime.seconds()) {
                robot.launchServo.setPosition(robot.launchliftStart);
            }

        telemetry.addData("Status", "Launcherlift Debug 1 at: " + runtime.toString());
        telemetry.addData("Servo", " 1 Launcherlift Push Set to " + robot.launchServo.getPosition());



        telemetry.addData("Status", " launchMotor Debug 1 at: " + runtime.toString());
        if (gamepad2.a && launcherState > 0 && (launchPress + 2) < runtime.seconds()) {
            robot.launchMotor.setPower(0);
            launcherState = 0;
            launchPress = runtime.seconds();

        }

        telemetry.addData("Motor", " 1 launchMotor Push Set to " + robot.launchMotor.getPower());

        if (gamepad2.a && launcherState == 0 && (launchPress + 2) < runtime.seconds()) {
            robot.launchMotor.setPower(1);
            launchPress = runtime.seconds();
            launcherState = 5;
            lastStateChange = runtime.milliseconds();

        }
        telemetry.addData("Motor", " 2 launchMotor Push Set to " + robot.launchMotor.getPower());

        if (gamepad2.b &&  (SpeedChange + 1) < runtime.seconds()) {
            robot.launchMotor.setPower(1);
            SpeedChange = runtime.seconds();
            robot.launchMotor.setMaxSpeed( robot.launchMotor.getMaxSpeed() - 100);

        }
        telemetry.addData("LaunchMotor:", "Max Speed Set to:" + robot.launchMotor.getMaxSpeed());


/*
        if (launcherState > 0 && (lastStateChange + 500) < runtime.milliseconds()) {

            if (launcherState < 5) {
                robot.launchMotor.setPower(robot.launchMotor.getPower() + .1);
                lastStateChange = runtime.milliseconds();
                launcherState++;
            }
            telemetry.addData("Motor", " 3 launchMotor Push Set to " + robot.launchMotor.getPower());

        }
*/
        if(Math.abs(gamepad2.right_stick_y) > .01) {
            robot.manipMotor.setPower(gamepad2.right_stick_y);
        }
        else {
            robot.manipMotor.setPower(0);
        }
    }  // loop


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.normalDrive(this, 0, 0);
    }

}