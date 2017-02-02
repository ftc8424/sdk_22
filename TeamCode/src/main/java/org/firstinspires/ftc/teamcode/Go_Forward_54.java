package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by FTC8424 on 1/21/2017.
 */
@Autonomous(name = "Go Forward 54", group = "Test")

public class Go_Forward_54 extends LinearOpMode {
    HardwareHelper robot = new HardwareHelper(FULLAUTO);
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.robot_init(hardwareMap);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        double  driveSpeed = .75;




        telemetry.addData("Init:" ,"Waiting for start");
        telemetry.update();
        waitForStart();


        //shooting one ball
        robot.autoLauncher(this, 0.52);
        //Moving 54 in6
        sleep(10000);
        robot.encoderDrive(this, driveSpeed, -65, -65, 10);

    }
}
