package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by avc on 11/23/2016.
 */

public class Auto_Blue extends LinearOpMode {

    HardwareHelper robot = new HardwareHelper(FULLAUTO);
    private ElapsedTime runtime = new ElapsedTime();

    @Override

    public void runOpMode() throws InterruptedException {
        robot.robot_init(hardwareMap);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        double  driveSpeed = .5;


        robot.robot_init(hardwareMap);

        robot.color.enableLed(true);
        idle();
        sleep(1000L);
        robot.color.enableLed(false);
        idle();

        telemetry.addData("Init:" ,"Waiting for start");
        telemetry.update();
        idle();
        waitForStart();


        // moving backward 6 in. in 2 sec. and starting launcher
        robot.encoderDrive(this, driveSpeed, -6, -6, 2);
        // shooting
        robot.autoLauncher(this);
        // backing up
        robot.encoderDrive(this, driveSpeed, -35, -35, 10);
        //turning to beacon (need to figure out 135 degree turn)
        robot.encoderDrive(this, driveSpeed, -30, 30, 2);

        //towards beacon
        robot.encoderDrive(this, driveSpeed, 67, 67, 10);
        //aligning towards beacon
        robot.encoderDrive(this, driveSpeed, -5, 5, 2);
        //towards the beacon
        robot.encoderDrive(this, driveSpeed, 18, 18, 5);


        //Pressing blue button, when we are blue alliance
        if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()) {
           robot.rightPush.setPosition(robot.rpushDeploy);
        } else if (robot.color.red() > 0 && robot.color.red() > robot.color.red()){
            robot.leftPush.setPosition(robot.lpushDeploy);
       } else if (robot.color.blue() == robot.color.red()){
            telemetry.addData("ColorDecision: ", "Not Pressing");
           telemetry.update();
       }
        idle();
        sleep(1000);
        robot.leftPush.setPosition(robot.lpushStart);
        robot.rightPush.setPosition(robot.rpushStart);
        idle();


        robot.encoderDrive(this, driveSpeed, -7, -7, 10);

        //Turning right towards beacon 2
        robot.encoderDrive(this, driveSpeed, -10, 10, 10);
        //Driving towards beacon 2
        robot.encoderDrive(this, driveSpeed, 52, 52, 10);
        //Turning left at Beacon 2
        robot.encoderDrive(this, driveSpeed, 12, -12, 10);
        //Driving Towards Beacon 2
        robot.encoderDrive(this, driveSpeed, 15, 15, 10);

        //pressing beacon 2 when we are blue alliance
        if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()) {
            robot.leftPush.setPosition(robot.lpushDeploy);
        } else if (robot.color.red() > 0 && robot.color.red() > robot.color.red()){
            robot.rightPush.setPosition(robot.rpushDeploy);
        } else if (robot.color.blue() == robot.color.red()){
            telemetry.addData("ColorDecision: ", "Not Pressing");
            telemetry.update();
        }
        idle();
        sleep(1000);
        robot.leftPush.setPosition(robot.lpushStart);
        robot.rightPush.setPosition(robot.rpushStart);
        idle();
    }
}
