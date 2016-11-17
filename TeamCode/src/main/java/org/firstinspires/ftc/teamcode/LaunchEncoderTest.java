package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLTELEOP;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.LAUNCHTEST;

/**
 * Created by FTC8424 on 11/16/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Launch_Test", group = "Sensor")
public class LaunchEncoderTest  extends LinearOpMode {

    HardwareHelper robot = new HardwareHelper(FULLTELEOP);



    @Override
    public void runOpMode() throws InterruptedException {

        robot.robot_init(hardwareMap);

        telemetry.addData("Init:" ,"Waiting for start");
        telemetry.update();
        idle();
        waitForStart();

       robot.launchMotor.setPower(1);
        sleep(1000);

        telemetry.addData("EncoderStart:", robot.launchMotor.getCurrentPosition() );
        telemetry.update();


        sleep(10000);

        telemetry.addData("EncoderEnd:", robot.launchMotor.getCurrentPosition() );
        telemetry.update();

        robot.launchMotor.setPower(0);
        sleep(10000);
    }
}