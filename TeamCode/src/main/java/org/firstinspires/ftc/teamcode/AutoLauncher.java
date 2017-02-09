//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;
//
//// Created by Mohana and Kim on 11/16/2016.
//@Autonomous(name = "Auto_Launcher", group = "Tests")
//@Disabled
//public class AutoLauncher extends LinearOpMode {
//
//    HardwareHelper robot = new HardwareHelper(FULLAUTO);
//
//    private double servoUpTime = 0;
//    private ElapsedTime runtime = new ElapsedTime();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.robot_init(hardwareMap);
//
//        telemetry.addData("Init:", "Waiting for start");
//        telemetry.update();
//        idle();
//        waitForStart();
//        robot.launchMotor1.setPower(1);
//        robot.launchMotor2.setPower(1);
//        telemetry.addData("Motor", "LaunchPower Set to " + robot.launchMotor1.getCurrentPosition());
//        telemetry.addData("Motor", "LaunchPower Set to " + robot.launchMotor2.getCurrentPosition());
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
//    }
//}
