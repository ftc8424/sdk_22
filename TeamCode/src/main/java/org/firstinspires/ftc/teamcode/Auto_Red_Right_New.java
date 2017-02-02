package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by FTC8424 on 1/14/2017.
 */



/**
 * Created by Mohana on 1/9/2017.
 */
@Autonomous(name = "Auto Red Right New", group = "RedSide")
public class Auto_Red_Right_New extends LinearOpMode {
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
        robot.encoderDrive(this, driveSpeed, 33, 33, 15);
        //45 degree turn
        robot.encoderDrive(this, driveSpeed, -2.9, 2.9, 15);
        robot.encoderDrive(this, driveSpeed, 56, 56, 15);
        robot.encoderDrive(this, driveSpeed, -4, 4, 10);
        robot.encoderDrive(this, driveSpeed, 8, 8, 10);
        //Logic for pressing beacon when we are blue alliance
        String button;
        if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()) {
            robot.rightPush.setPosition(robot.rpushDeploy);
            button = "Left/Blue";
        } else if (robot.color.red() > 0 && robot.color.red() > robot.color.blue()) {
            robot.leftPush.setPosition(robot.lpushDeploy);
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
        robot.encoderDrive(this, 0.5, -7.5, -7.5, 10);
        //Turning right towards beacon 2
        robot.encoderDrive(this, driveSpeed, 13, -13, 10);
        //Driving towards beacon 2
        robot.encoderDrive(this, driveSpeed, 45.25, 45.25, 10);
        //Turning left at Beacon 2
        robot.encoderDrive(this, driveSpeed, -13, 13, 10);
        //Moving forward to get close enough to hit the beacon
        robot.encoderDrive(this, driveSpeed, 5.25, 5.25, 10);
        //Pressing blue button, when we are blue alliance
        button = "Not Pressing";
        if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()) {
            robot.rightPush.setPosition(robot.rpushDeploy);
            button = "Right/Blue";
        } else if (robot.color.red() > 0 && robot.color.red() > robot.color.blue()) {
            robot.leftPush.setPosition(robot.lpushDeploy);
            button = "Left/Red";
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


        //backing up from beacon 2
        robot.encoderDrive(this, driveSpeed, -7, -7, 8);
        //aligning (turning) to shoot
        robot.encoderDrive(this, driveSpeed, 6.5, -6.5, 5);
        //shooting
        robot.autoLauncher(this, 0.52);
        //forward to capball and parking on center
        robot.encoderDrive(this, driveSpeed, -60, -60, 10);

    }



}


