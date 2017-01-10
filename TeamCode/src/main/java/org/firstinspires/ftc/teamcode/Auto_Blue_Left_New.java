package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by Mohana on 1/9/2017.
 */

public class Auto_Blue_Left_New extends LinearOpMode {
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
        robot.color.enableLed(true);
        sleep(1000L);
        robot.color.enableLed(false);
        waitForStart();


        //Set up at a 45 degree angle
        //Moving forward toward beacon
        robot.encoderDrive(this, driveSpeed, -75, -75, 15);
        //45 degree turn
        robot.encoderDrive(this, driveSpeed, 19.5, -19.5, 10);
        robot.encoderDrive(this, driveSpeed, 16, 16, 10);
        //Logic for pressing beacon when we are blue alliance
        String button;
        if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()) {
            robot.leftPush.setPosition(robot.lpushDeploy);
            button = "Left/Blue";
        } else if (robot.color.red() > 0 && robot.color.red() > robot.color.blue()) {
            robot.rightPush.setPosition(robot.rpushDeploy);
            button = "Right/Blue";
        } else {
            button = String.format("Not Pressing: Blue %d / Red %d",
                    robot.color.blue(), robot.color.red());
        }
        telemetry.addData("Pressing: %s", button);
        telemetry.update();
        sleep(1000);
        if ( !opModeIsActive() ) return;

        robot.leftPush.setPosition(robot.lpushStart);
        robot.rightPush.setPosition(robot.rpushStart);


        //backing up from beacon 1
        robot.encoderDrive(this, 0.5, -7, -7, 10);
        //Turning right towards beacon 2
        robot.encoderDrive(this, driveSpeed, -13, 13, 10);
        //Driving towards beacon 2
        robot.encoderDrive(this, driveSpeed, 44, 44, 10);
        //Turning left at Beacon 2
        robot.encoderDrive(this, driveSpeed, 13, -13, 10);
        //Moving forward to get close enough to hit the beacon
        robot.encoderDrive(this, driveSpeed, 9, 9, 10);
        //Pressing blue button, when we are blue alliance
        button = "Not Pressing";
        if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()) {
            robot.leftPush.setPosition(robot.lpushDeploy);
            button = "Left/Blue";
        } else if (robot.color.red() > 0 && robot.color.red() > robot.color.blue()) {
            robot.rightPush.setPosition(robot.rpushDeploy);
            button = "Right/Blue";
        } else {
            button = String.format("Not Pressing: Blue %d / Red %d",
                    robot.color.blue(), robot.color.red());
        }
        telemetry.addData("Pressing: %s", button);
        telemetry.update();
        sleep(1000);
        if ( !opModeIsActive() ) return;

        robot.leftPush.setPosition(robot.lpushStart);
        robot.rightPush.setPosition(robot.rpushStart);
        if ( !opModeIsActive() ) return;

        //Resetting button pushers to starting position
        robot.leftPush.setPosition(robot.lpushStart);
        robot.rightPush.setPosition(robot.rpushStart);

        //backing up from beacon 2
        robot.encoderDrive(this, driveSpeed, -7, -7, 8);
        //aligning (turning) to shoot
        robot.encoderDrive(this, driveSpeed, -6.5, 6.5, 5);
        //shooting
        robot.autoLauncher(this, 0.58);
        //forward to capball and parking on center
        robot.encoderDrive(this, driveSpeed, -20, -20, 10);
        
    }



}

