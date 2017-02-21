package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

// Created by Mohana on 11/16/2016.
@Autonomous(name = "TestGyro", group = "Tests")
//@Disabled
public class TestGyro extends LinearOpMode {

    HardwareHelper robot = new HardwareHelper(FULLAUTO);

    private double servoUpTime = 0;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.robot_init(hardwareMap);

        telemetry.addData("Init:", "Gyro Waiting for initialize");
        robot.gyro.calibrate();
        do {
            telemetry.addData("Gyro:", "Calibrating");
            telemetry.update();
            idle();
        } while (!this.isStopRequested() && robot.gyro.isCalibrating());

        telemetry.addData("Init:", "Waiting for Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Heading:  ", robot.gyro.getHeading());
            telemetry.update();
            idle();
        }
        telemetry.addData("Heading:  ", "Final Heading %d", robot.gyro.getHeading());

//        robot.launchMotor.setPower(1);
//        telemetry.addData("Motor", "LaunchPower Set to " + robot.launchMotor.getCurrentPosition());
//
//        sleep(2500);
//        if ( !opModeIsActive() ) return;
//        telemetry.addData("Status", "Debug 1 at: " + runtime.toString());
//        robot.launchServo.setPosition(robot.launchliftDeploy);
//        sleep(500);
//        if ( !opModeIsActive() ) return;
//        robot.launchServo.setPosition(robot.launchliftStart);
//        sleep(750);
//        if ( !opModeIsActive() ) return;
//        robot.launchServo.setPosition(robot.launchliftDeploy);
//        sleep(500);
//        if ( !opModeIsActive() ) return;
//        robot.launchServo.setPosition(robot.launchliftStart);
//        sleep(3000);
    }
}
