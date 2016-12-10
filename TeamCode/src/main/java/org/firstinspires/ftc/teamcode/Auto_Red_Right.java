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
@Autonomous(name = "Auto Red Right", group = "RedSide")
public class Auto_Red_Right extends LinearOpMode {

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
        robot.encoderDrive(this, driveSpeed, -20, -20, 5);
        //shooting 2 balls in to vortex
        robot.autoLauncher(this);
        //Moving forward to center itself inside of the nearest mat
        robot.encoderDrive(this, driveSpeed, -7, -7, 5);
        //45 degree turn aligning itself to
        robot.encoderDrive(this, driveSpeed, -6, 6, 5);
        //Driving towards the corner vortex
        robot.encoderDrive(this, driveSpeed, -37, -37, 3);
        // 135 degree turn to start to line up for beacon press
        robot.encoderDrive(this, driveSpeed, -19.5, 19.5, 10);
        //Moving forwards toward beacon
        robot.encoderDrive(this, driveSpeed, 45, 45, 6);
        //Turning to align at beacon
        robot.encoderDrive(this, driveSpeed, -5.5, 5.5, 10);
        //Move forward toward the beacon to be able to hit beacon
        robot.encoderDrive(this, driveSpeed, 2, 2, 5);



        //Pressing red button, when we are red alliance
        String button;
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
        robot.encoderDrive(this, driveSpeed, -7, -7, 10);
        //Turning right towards beacon 2
        robot.encoderDrive(this, driveSpeed, 13, -13, 10);
        //Driving towards beacon 2
        robot.encoderDrive(this, driveSpeed, 46, 46, 10);
        //Turning left at Beacon 2
        robot.encoderDrive(this, driveSpeed, -13, 13, 10);
        //Moving forward to get close enough to hit the beacon
        robot.encoderDrive(this, driveSpeed, 7, 7, 10);

        //Pressing red button, when we are red alliance
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

        robot.encoderDrive(this, driveSpeed, -7, -7, 10);
        robot.encoderDrive(this, driveSpeed, 5, -5, 10);
        robot.encoderDrive(this, driveSpeed, -53, -53, 10);

        telemetry.addData("Pressing: %s", button);
        telemetry.update();
        sleep(1000);
        if ( !opModeIsActive() ) return;

        //Resetting button pushers to starting position
        robot.leftPush.setPosition(robot.lpushStart);
        robot.rightPush.setPosition(robot.rpushStart);

        /*
         * Now need to back up, hit the cap-ball and go park on the ramp
         */
    }
}


