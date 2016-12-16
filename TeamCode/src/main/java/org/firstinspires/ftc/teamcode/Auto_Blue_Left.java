package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by FTC8424 on 12/8/2016.
 */



/**
 * Created by Devan on 10/9/2016.
 */
@Autonomous(name = "Auto Blue Left", group = "BlueSide")
public class Auto_Blue_Left extends LinearOpMode {

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

        //Autonomous starting on the 4th square from the red corner vortex
        //Moving forward to shoot at vortex
        robot.encoderDrive(this, 0.5, -10, -10, 5);
        robot.encoderDrive(this, 0.5, 6, -6, 5);
        robot.encoderDrive(this, 0.5, -19, -19, 5);
        robot.encoderDrive(this, 0.5, 6, -6, 5);

        robot.encoderDrive(this, 0.5, -30, -30, 3);
        // 135 degree turn to start to line up for beacon press
        robot.encoderDrive(this, driveSpeed, 19.5, -19.5, 10);
        //Moving forwards toward beacon
        robot.encoderDrive(this, driveSpeed, 39, 39, 6);
        //Turning to align at
        robot.encoderDrive(this, driveSpeed, 4.5, -4.5, 10);
        //Moving towards beacon
        robot.encoderDrive(this, driveSpeed, 12, 12, 10);


        //Pressing blue button, when we are blue alliance
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




//        robot.encoderDrive(this, 0.5, -20, -20, 5);
//        //shooting 2 balls in to vortex
//        //robot.autoLauncher(this);
//        //Moving forward to center itself inside of the nearest mat
////        robot.encoderDrive(this, 0.65, -8, -8, 5);
//        //45 degree turn aligning itself to
//        robot.encoderDrive(this, driveSpeed, 6, -6, 5);
//        //Driving towards the corner vortex
//        robot.encoderDrive(this, driveSpeed, -30, -30, 3);
//        // 135 degree turn to start to line up for beacon press
//        robot.encoderDrive(this, driveSpeed, 19.5, -19.5, 10);
//        //Moving forwards toward beacon
//        robot.encoderDrive(this, driveSpeed, 36, 36, 6);
//        //Turning to align at
//        robot.encoderDrive(this, driveSpeed, 5.5, -5.5, 10);
//        //Moving towards beacon
//        robot.encoderDrive(this, driveSpeed, 15, 15, 10);
//
//
//        //Pressing blue button, when we are blue alliance
//        if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()) {
//            robot.leftPush.setPosition(robot.lpushDeploy);
//            button = "Left/Blue";
//        } else if (robot.color.red() > 0 && robot.color.red() > robot.color.blue()) {
//            robot.rightPush.setPosition(robot.rpushDeploy);
//            button = "Right/Blue";
//        } else {
//            button = String.format("Not Pressing: Blue %d / Red %d",
//                    robot.color.blue(), robot.color.red());
//        }
//        telemetry.addData("Pressing: %s", button);
//        telemetry.update();
//        sleep(1000);
//        if ( !opModeIsActive() ) return;
//
//        robot.leftPush.setPosition(robot.lpushStart);
//        robot.rightPush.setPosition(robot.rpushStart);
        //backing up from beacon 1
        robot.encoderDrive(this, 0.5, -7, -7, 10);
        //Turning right towards beacon 2
        robot.encoderDrive(this, driveSpeed, -12, 12, 10);
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

        /*
         * Now need to back up, hit the cap-ball and go park on the ramp
         */
    }
}


