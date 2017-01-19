package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by Devan on 10/9/2016.
 */
@Autonomous(name = "Auto Blue Right", group = "BlueSide")
@Disabled
public class Auto_Blue_Right extends LinearOpMode {

    HardwareHelper robot = new HardwareHelper(FULLAUTO);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.robot_init(hardwareMap);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        double  driveSpeed = .75;


        robot.robot_init(hardwareMap);

        robot.color.enableLed(true);
        sleep(1000L);
        robot.color.enableLed(false);

        telemetry.addData("Init:" ,"Waiting for start");
        telemetry.update();
        waitForStart();

        // Shoot the particles as the first thing
        robot.autoLauncher(this, 0.58);
        //forwards 18 in
        robot.encoderDrive(this, driveSpeed, -23, -23, 3);
        //turning left to beacon
        robot.encoderDrive(this, driveSpeed, -18, 18, 5);
        //towards beacon
        robot.encoderDrive(this, driveSpeed, 38, 38, 10);
        //aligning towards beacon
        robot.encoderDrive(this, driveSpeed, 6 , -6, 2);
        //towards the beacon
        robot.encoderDrive(this, driveSpeed, 21, 21, 10);

        //Pressing blue button, when we are blue alliance
        String button;
        if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()) {
            robot.leftPush.setPosition(robot.lpushDeploy);
            button = "Right/Blue";
        } else if (robot.color.red() > 0 && robot.color.red() > robot.color.blue()) {
            robot.rightPush.setPosition(robot.rpushDeploy);
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
        robot.encoderDrive(this, driveSpeed, -7, -7, 10);
        //Turning right towards beacon 2
        robot.encoderDrive(this, driveSpeed, -13, 13, 10);
        //Driving towards beacon 2
        robot.encoderDrive(this, driveSpeed, 43, 43, 10);
        //Turning left at Beacon 2
        robot.encoderDrive(this, driveSpeed, 13, -13, 10);
        //Driving Towards Beacon 2
        robot.encoderDrive(this, driveSpeed, 9, 9, 10);

        //pressing beacon 2 when we are blue alliance
        button = "Not Pressing";
        if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()) {
            robot.leftPush.setPosition(robot.lpushDeploy);
            button = "Right/Blue";
        } else if (robot.color.red() > 0 && robot.color.red() > robot.color.blue()) {
            robot.rightPush.setPosition(robot.rpushDeploy);
            button = "Left/Red";
        } else {
            button = String.format("Not Pressing: Blue %d / Red %d",
                    robot.color.blue(), robot.color.red());
        }

        robot.encoderDrive(this, driveSpeed, -7, -7, 10);
        robot.encoderDrive(this, driveSpeed, -5, 5, 10);
        robot.encoderDrive(this, driveSpeed, -53, -53, 10);

        telemetry.addData("Pressing: %s", button);
        telemetry.update();
        sleep(1000);
        if ( !opModeIsActive() ) return;

        robot.leftPush.setPosition(robot.lpushStart);
        robot.rightPush.setPosition(robot.rpushStart);

        if ( !opModeIsActive() ) return;

        robot.leftPush.setPosition(robot.lpushStart);
        robot.rightPush.setPosition(robot.rpushStart);

        /*
         * Now need to back up, hit the cap-ball and go park on the ramp
         */
    }
}
