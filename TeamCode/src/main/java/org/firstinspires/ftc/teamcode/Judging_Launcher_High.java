//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;
//
///**
// * Created by FTC8424 on 1/14/2017.
// */
//@Autonomous(name = "Judging_Launcher_High", group = "Judge")
//
//public class Judging_Launcher_High extends LinearOpMode {
//
//    HardwareHelper robot = new HardwareHelper(FULLAUTO);
//
//    private double servoUpTime = 0;
//    private ElapsedTime runtime = new ElapsedTime();
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        robot.robot_init(hardwareMap);
//
//        telemetry.addData("Init:", "Waiting for start");
//        telemetry.update();
//        idle();
//        waitForStart();
//
//        robot.launchMotor1.setPower(0.75);
//        telemetry.addData("Motor", "LaunchPower Set to " + robot.launchMotor1.getCurrentPosition());
//        robot.launchMotor2.setPower(0.75);
//        telemetry.addData("Motor", "LaunchPower Set to " + robot.launchMotor2.getCurrentPosition());
//
////        sleep(2500);
////        if ( !opModeIsActive() ) return;
////        telemetry.addData("Status", "Debug 1 at: " + runtime.toString());
////        robot.launchServo.setPosition(robot.launchliftDeploy);
////        sleep(500);
//
//
//    }
//}
