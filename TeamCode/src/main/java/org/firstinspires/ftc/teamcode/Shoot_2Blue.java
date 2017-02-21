package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by FTC8424 on 2/18/2017.
 */


/**
 * Created by FTC8424 on 1/21/2017.
 */
@Autonomous(name = "Shoot_2Blue", group = "Test")

public class Shoot_2Blue extends LinearOpMode {
    HardwareHelper robot = new HardwareHelper(FULLAUTO);
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.robot_init(hardwareMap);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        double  driveSpeed = .4;




        telemetry.addData("Init:" ,"Waiting for start");
        telemetry.update();
        waitForStart();

        sleep(10000);

        robot.encoderDrive(this, driveSpeed, -30, -30, 10);

        robot.autoLauncher(this, 0.65);
        //forward to capball and parking on center
        robot.encoderDrive(this, driveSpeed, -14, -14, 10);
    }
}


