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
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLTELEOP;

@TeleOp(name="Motor Encoder Tester", group="Tests")  // @Autonomous(...) is the other common choice

public class EncoderTests extends OpMode {


    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private HardwareHelper robot = new HardwareHelper(FULLTELEOP);

    private HashMap<String, Double> motorTimes = new HashMap<String, Double>();
    private HashMap<String, Integer> motorEncoder = new HashMap<String, Integer>();
    private int motorState = 0;
    private double launchPress = 0;
    private double launchStart = 0;
    private double drivePress = 0;
    private double driveStart = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        motorTimes.put("LM-Start", 0.0);
        motorTimes.put("LM-End", 0.0);
        motorTimes.put("Drives-Start", 0.0);
        motorTimes.put("Drives-End", 0.0);
        motorEncoder.put("LM-Start", 0);
        motorEncoder.put("LM-End", 0);
        motorEncoder.put("LB-Start", 0);
        motorEncoder.put("LB-End", 0);
        motorEncoder.put("RB-Start", 0);
        motorEncoder.put("RB-End", 0);
        motorEncoder.put("LMM-Start", 0);
        motorEncoder.put("LMM-End", 0);
        motorEncoder.put("RMM-Start", 0);
        motorEncoder.put("RMM-End", 0);
        robot.robot_init(hardwareMap);
        boolean resetOk =  robot.waitForReset(robot.leftBackDrive, robot.rightBackDrive,
                robot.leftMidDrive, robot.rightMidDrive, 2000);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftMidDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftMidDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "Initialized and encoders %s reset", resetOk ? "ARE" : "ARE NOT");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    @Override
    public void init_loop() {
        //telemetry.addData("Status", "Waiting for Play: ", runtime.toString());
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

        switch (motorState) {
            case 0:
                if (gamepad1.a && launchPress + 10000 < runtime.milliseconds()) {
                    robot.launchMotor.setPower(1.0);
                    launchPress = runtime.milliseconds();
                    motorState = 1;
                } else if ( gamepad1.y && launchPress + 10000 < runtime.milliseconds() ) {
                    robot.leftBackDrive.setPower(1.0);
                    robot.leftMidDrive.setPower(1.0);
                    robot.rightBackDrive.setPower(1.0);
                    robot.rightMidDrive.setPower(1.0);
                    launchPress = runtime.milliseconds();
                    motorState = 3;
                }
                break;

            case 1:
                if (launchPress + 2000 < runtime.milliseconds()) {
                    launchStart = runtime.milliseconds();
                    motorEncoder.put("LM-Start", robot.launchMotor.getCurrentPosition());
                    motorTimes.put("LM-Start", launchStart);
                    motorState = 2;
                }
                break;

            case 2:
                if (launchStart + 10000 < runtime.milliseconds()) {
                    motorTimes.put("LM-End", runtime.milliseconds());
                    robot.launchMotor.setPower(0.0);
                    motorEncoder.put("LM-End", robot.launchMotor.getCurrentPosition());
                    motorState = 0;
                }
                break;

            case 3:
                if ( launchPress + 2000 < runtime.milliseconds() ) {
                    launchStart = runtime.milliseconds();
                    motorTimes.put("Drives-Start", launchStart);
                    motorEncoder.put("LB-Start", robot.leftBackDrive.getCurrentPosition());
                    motorEncoder.put("RB-Start", robot.rightBackDrive.getCurrentPosition());
                    motorEncoder.put("LMM-Start", robot.leftMidDrive.getCurrentPosition());
                    motorEncoder.put("RMM-Start", robot.rightMidDrive.getCurrentPosition());
                    motorState = 4;
                }
                break;

            case 4:
                if ( launchStart + 10000 < runtime.milliseconds() ) {
                    motorTimes.put("Drives-End", runtime.milliseconds());
                    robot.normalDrive(this, 0.0, 0.0);
                    motorEncoder.put("LB-End", robot.leftBackDrive.getCurrentPosition());
                    motorEncoder.put("RB-End", robot.rightBackDrive.getCurrentPosition());
                    motorEncoder.put("LMM-End", robot.leftMidDrive.getCurrentPosition());
                    motorEncoder.put("RMM-End", robot.rightMidDrive.getCurrentPosition());
                    motorState = 0;
                }
        }
        telemetry.addData("LM-Runtime:", "%d in %.2f secs",
                        motorEncoder.get("LM-End") - motorEncoder.get("LM-Start"),
                        (motorTimes.get("LM-End") - motorTimes.get("LM-Start")) / 1000.0)
                .addData("Drives Runtime:", "%.2f secs",
                        (motorTimes.get("Drives-End") - motorTimes.get("Drives-Start")) / 1000.0)
                .addData("LB-Encoders:", "%d",
                        motorEncoder.get("LB-End") - motorEncoder.get("LB-Start"))
                .addData("RB-Encoders:", "%d",
                        motorEncoder.get("RB-End") - motorEncoder.get("RB-Start"))
                .addData("LMM-Encoders:", "%d",
                        motorEncoder.get("LMM-End") - motorEncoder.get("LMM-Start"))
                .addData("RMM-Encoders:", "%d",
                        motorEncoder.get("LB-End") - motorEncoder.get("LB-Start"));
    }  // loop


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.normalDrive(this, 0, 0);
    }

}