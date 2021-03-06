package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by FTC8424 on 1/14/2017.
 */
@Autonomous(name = "Auto Blue Right Gyro", group = "BlueSide")
public class Auto_Blue_Right_Gyro extends LinearOpMode {
    HardwareHelper robot = new HardwareHelper(FULLAUTO);
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        robot.robot_init(hardwareMap);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        double driveSpeed = .4;
        telemetry.addData("Init:" ,"Waiting for start");
        telemetry.update();
        robot.color.enableLed(true);
        robot.color.enableLed(false);
        robot.gyro.calibrate();
        while(!isStopRequested() && robot.gyro.isCalibrating()){
            telemetry.addData("Init:", "Calibrating");
            telemetry.update();
        }
        telemetry.addData("Init:", "Calibrated!!");
        telemetry.update();
        waitForStart();
// 24
        robot.encoderDrive(this, driveSpeed, 22, 22, 10);

        //robot.encoderDrive(this, 0.4, -6.9, 6.9, 10);
       robot.gyroTurn(this, 45, 10);

        robot.encoderDrive(this, driveSpeed, 39, 39, 10);
        //45 degree turn
        //sleep(1000);
        robot.gyroTurn(this, 85, 5);

        //robot.encoderDrive(this, 0.35, -4.5, 4.5, 10);
        robot.encoderDrive(this, driveSpeed, 17, 17, 10);
        //logic for pressing button

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

        robot.encoderDrive(this, driveSpeed, 2.5, 2.5, 5);
//        sleep(1000);
        if ( !opModeIsActive() ) return;
        robot.encoderDrive(this, driveSpeed, -11, -11, 10);

        robot.leftPush.setPosition(robot.lpushStart);
        robot.rightPush.setPosition(robot.rpushStart);

        //robot.gyroTurn(this, 95, 5);
        robot.autoLauncher(this, 0.65);

        //Turning right towards beacon 2
        robot.gyroTurn(this, 0, 5);
        //robot.encoderDrive(this, 0.25, 12, -12, 10);
        //Driving towards beacon 2
        robot.encoderDrive(this, driveSpeed, 45.5, 45.5, 10);
        //Turning left at Beacon 2
        robot.gyroTurn(this, 88, 5);

        //robot.encoderDrive(this, 0.25, -12.5, 12.5, 10);
        //Moving forward to get close enough to hit the beacon
        robot.encoderDrive(this, driveSpeed, 6, 6, 10);

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
        telemetry.addData("Pressing: %s", button);
        telemetry.update();

        //Resetting button pushers to starting position

        robot.encoderDrive(this, driveSpeed, 2, 2, 10);

        if ( !opModeIsActive() ) return;

        //backing up from beacon 2
        robot.encoderDrive(this, driveSpeed, -10.5, -10.5, 8);
//        robot.leftPush.setPosition(robot.lpushStart);
//        robot.rightPush.setPosition(robot.rpushStart);

        //aligning (turning) to knock cap ball off
//        robot.gyroTurn(this, 315, 5);
        robot.encoderDrive(this, 1.0, -7.5, 7.5, 5);
       // robot.encoderDrive(this, driveSpeed, 6.5, -6.5, 5);
        // Forward to capball and parking on center
        robot.encoderDrive(this, 1.0, -50.5, -50.5, 10);
    }
}
