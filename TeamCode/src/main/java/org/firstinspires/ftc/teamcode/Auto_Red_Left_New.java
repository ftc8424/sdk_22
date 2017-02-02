package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by FTC8424 on 1/14/2017.
 */
@Autonomous(name = "Auto Red Left New", group = "RedSide")
public class Auto_Red_Left_New extends LinearOpMode {
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

        robot.encoderDrive(this, driveSpeed, 65, 65, 10);
        //45 degree turn
        sleep(1000);
        robot.encoderDrive(this, driveSpeed, -6.9, 6.9, 10);
        robot.encoderDrive(this, driveSpeed, 17, 17, 10);
        //logic for pressing button
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



        robot.encoderDrive(this, driveSpeed, -8.5, -8.5, 10);
        //Turning right towards beacon 2
        robot.encoderDrive(this, driveSpeed, 12, -12, 10);
        //Driving towards beacon 2
        robot.encoderDrive(this, driveSpeed, 45, 45, 10);
        //Turning left at Beacon 2
        robot.encoderDrive(this, driveSpeed, -12, 12, 10);
        //Moving forward to get close enough to hit the beacon
        robot.encoderDrive(this, driveSpeed, 6, 6, 10);

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

        //Resetting button pushers to starting position
        robot.leftPush.setPosition(robot.lpushStart);
        robot.rightPush.setPosition(robot.rpushStart);


        //backing up from beacon 2
        robot.encoderDrive(this, driveSpeed, -9, -9, 8);
        //aligning (turning) to shoot
        robot.encoderDrive(this, driveSpeed, 6.5, -6.5, 5);
        //shooting
        robot.autoLauncher(this, 0.5);
        //forward to capball and parking on center
        robot.encoderDrive(this, driveSpeed, -60, -60, 10);

    }
}
