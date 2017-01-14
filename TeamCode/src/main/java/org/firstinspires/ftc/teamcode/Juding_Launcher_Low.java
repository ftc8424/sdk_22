package org.firstinspires.ftc.teamcode;

/**
 * Created by FTC8424 on 1/14/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

@Autonomous(name = "Judging_Launcher_Low", group = "Judge")

public class Juding_Launcher_Low extends LinearOpMode{

    HardwareHelper robot = new HardwareHelper(FULLAUTO);

    private double servoUpTime = 0;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.robot_init(hardwareMap);

        telemetry.addData("Init:", "Waiting for start");
        telemetry.update();
        idle();
        waitForStart();


        robot.launchMotor.setPower(0.35);
        telemetry.addData("Motor", "LaunchPower Set to " + robot.launchMotor.getCurrentPosition());

        sleep(2500);
        if ( !opModeIsActive() ) return;
        telemetry.addData("Status", "Debug 1 at: " + runtime.toString());
        robot.launchServo.setPosition(robot.launchliftDeploy);
        sleep(500);



    }
}
