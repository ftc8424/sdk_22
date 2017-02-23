package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by FTC8424 on 1/14/2017.
 */
@Autonomous(name = "Auto Drive Forward", group = "Test")
public class Auto_DriveForward extends LinearOpMode {
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
        robot.encoderDrive(this, driveSpeed, 10*12, 10*12, 10);
//
//        //robot.encoderDrive(this, 0.4, -6.9, 6.9, 10);
//       robot.gyroTurn(this, 315, 10);
//
//        robot.encoderDrive(this, driveSpeed, 37.5, 37.5, 10);
//        //45 degree turn
//        //sleep(1000);
//        robot.gyroTurn(this, 270, 5);
//
//        //robot.encoderDrive(this, 0.35, -4.5, 4.5, 10);
//        robot.encoderDrive(this, driveSpeed, 15, 15, 10);
//        //logic for pressing button
//
//        String button;
//        if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()) {
//            robot.rightPush.setPosition(robot.rpushDeploy);
//            button = "Left/Blue";
//        } else if (robot.color.red() > 0 && robot.color.red() > robot.color.blue()) {
//            robot.leftPush.setPosition(robot.lpushDeploy);
//            button = "Right/Blue";
//        } else {
//            button = String.format("Not Pressing: Blue %d / Red %d",
//                    robot.color.blue(), robot.color.red());
//        }
//        telemetry.addData("Pressing: %s", button);
//        telemetry.update();
//
//        robot.encoderDrive(this, driveSpeed, 2, 2, 5);
//        sleep(1000);
//        if ( !opModeIsActive() ) return;
//
//        robot.leftPush.setPosition(robot.lpushStart);
//        robot.rightPush.setPosition(robot.rpushStart);
//
//
//        robot.encoderDrive(this, driveSpeed, -10, -10, 10);
//
//        robot.gyroTurn(this, 265, 5);
//        robot.autoLauncher(this, 0.65);
//
//        //Turning right towards beacon 2
//        robot.gyroTurn(this, 350, 5);
//        //robot.encoderDrive(this, 0.25, 12, -12, 10);
//        //Driving towards beacon 2
//        robot.encoderDrive(this, driveSpeed, 47.5, 47.5, 10);
//        //Turning left at Beacon 2
//        robot.gyroTurn(this, 270, 5);
//
//        //robot.encoderDrive(this, 0.25, -12.5, 12.5, 10);
//        //Moving forward to get close enough to hit the beacon
//        robot.encoderDrive(this, driveSpeed, 3.5, 3.5, 10);
//
//        button = "Not Pressing";
//        if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()) {
//            robot.rightPush.setPosition(robot.rpushDeploy);
//            button = "Right/Blue";
//        } else if (robot.color.red() > 0 && robot.color.red() > robot.color.blue()) {
//            robot.leftPush.setPosition(robot.lpushDeploy);
//            button = "Left/Red";
//        } else {
//            button = String.format("Not Pressing: Blue %d / Red %d",
//                    robot.color.blue(), robot.color.red());
//        }
//        telemetry.addData("Pressing: %s", button);
//        telemetry.update();
//
//
//        //Resetting button pushers to starting position
//
//        robot.encoderDrive(this, driveSpeed, 1, 1, 10);
//        sleep(1000);
//        if ( !opModeIsActive() ) return;
//
//        robot.leftPush.setPosition(robot.lpushStart);
//        robot.rightPush.setPosition(robot.rpushStart);
//
//        //backing up from beacon 2
//        robot.encoderDrive(this, driveSpeed, -10.5, -10.5, 8);
//        //aligning (turning) to shoot
//        robot.gyroTurn(this, 315, 5);
//       // robot.encoderDrive(this, driveSpeed, 6.5, -6.5, 5);
//        //shooting
//        robot.encoderDrive(this, driveSpeed, -31.5, -31.5, 10);
//
//        //forward to capball and parking on center
//        robot.encoderDrive(this, driveSpeed, -14, -14, 10);


    }
}
