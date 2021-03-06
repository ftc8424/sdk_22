package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by Mohana on 1/7/2017.
 */
@Autonomous(name = "Auto Red Right", group = "RedSide")
public class Auto_Red_Right_Capball extends LinearOpMode {
    HardwareHelper robot = new HardwareHelper(FULLAUTO);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.robot_init(hardwareMap);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        double driveSpeed = .75;
        telemetry.addData("Init:" ,"Waiting for start");
        telemetry.update();
        waitForStart();




        sleep(10000);
        robot.encoderDrive(this, driveSpeed, -56, -56, 10);
    }

}

