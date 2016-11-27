package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLTELEOP;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.LAUNCHTEST;

/**
 * Created by FTC8424 on 11/16/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Launch_Test", group = "Sensor")
public class LaunchEncoderTest  extends LinearOpMode {

    HardwareHelper robot = new HardwareHelper(FULLAUTO);



    @Override
    public void runOpMode() throws InterruptedException {

        robot.robot_init(hardwareMap);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftMidDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Init:" ,"Waiting for start");
        telemetry.update();
        idle();
        waitForStart();

       robot.leftBackDrive.setPower(1);
        robot.rightBackDrive.setPower(1);
        robot.leftMidDrive.setPower(1);
        robot.rightMidDrive.setPower(1);


        sleep(1000);

        int LB = robot.leftBackDrive.getCurrentPosition();
        int RB = robot.rightBackDrive.getCurrentPosition();
        int LM = robot.leftMidDrive.getCurrentPosition();
        int RM = robot.rightMidDrive.getCurrentPosition();



        telemetry.addData("Drives", "Currently At %7d :%7d %7d : %7d",
                LB,
                LM,
                RB,
                RM);

        telemetry.update();


        sleep(10000);

        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.leftMidDrive.setPower(0);
        robot.rightMidDrive.setPower(0);
        LB = robot.leftBackDrive.getCurrentPosition();
        RB = robot.rightBackDrive.getCurrentPosition();
         LM = robot.leftMidDrive.getCurrentPosition();
         RM = robot.rightMidDrive.getCurrentPosition();

        telemetry.addData("Drives End", "Currently At %7d :%7d %7d : %7d",
                LB,
                LM,
                RB,
                RM);

        telemetry.update();

        sleep(10000);


    }
}