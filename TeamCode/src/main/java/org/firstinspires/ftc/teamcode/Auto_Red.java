package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
//
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.AUTOTEST;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by Devan on 10/9/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto_Red", group = "Sensor")
public class Auto_Red extends LinearOpMode{
//Trollbot is 14.5 inches

    HardwareHelper robot = new HardwareHelper(FULLAUTO);

    @Override
    public void runOpMode() throws InterruptedException {

//        waitForStart();
//
//     hsvValues is an array that will hold the hue, saturation, and value information.
//        float hsvValues[] = {0F, 0F, 0F};
//
//    // values is a reference to the hsvValues array.
//        final float values[] = hsvValues;

//        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
//
//    // bPrevState and bCurrState represent the previous and current state of the button.
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



//        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
//
//      // send the info back to driver station using telemetry function.

        //Driving towards center vortex knocking off cap ball in 1 qualifier
        robot.encoderDrive(this, driveSpeed, 40, 40, 5);
        //40 for real autonomous
        //Turning towards the beacon
        sleep(1000);
       robot.encoderDrive(this, .25, -9.75, 9.75, 5);
        //6.5
        //Driving towards the 1st beacon
        robot.encoderDrive(this, driveSpeed, 67, 67, 5);
        //44.6666
        //Aligning the robot at the 1st beacon
        robot.encoderDrive(this, driveSpeed, -5, 5, 5);
        //3.33
        //Driving towards hte first beacon
        robot.encoderDrive(this, driveSpeed, 18, 18, 5);
        //12



        telemetry.addData("Auto: ", "Drive completed, looking for color sensor");
        telemetry.update();
        //this.sleep(5000);
        telemetry.addData("Color:" , "Connection Info %s ", robot.color.getConnectionInfo());
        telemetry.update();


        telemetry.addData("Color: ", "Red %d Blue %d Green %d", robot.color.red(), robot.color.blue(), robot.color.green());
        telemetry.update();
//
//        telemetry.addData("Clear", robot.color.alpha());
//        telemetry.update();
//        telemetry.addData("Red  ", robot.color.red());
//        telemetry.update();
//        telemetry.addData("Green", robot.color.green());
//        telemetry.update();
//        telemetry.addData("Blue ", robot.color.blue());
//        telemetry.update();

        //Logic for pressing the red button, when we are the red alliance (at the 1st beacon)
        if (robot.color.red() > 0 && robot.color.red() > robot.color.blue()) {
            robot.leftPush.setPosition(robot.lpushDeploy);
        } else if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()){
            robot.rightPush.setPosition(robot.rpushDeploy);
        } else if (robot.color.red() == robot.color.blue()){
            telemetry.addData("ColorDecision: ", "Not Pressing");
            telemetry.update();
        }
        idle();
        sleep(1000);
        robot.leftPush.setPosition(robot.lpushStart);
        robot.rightPush.setPosition(robot.rpushStart);
        idle();

        robot.encoderDrive(this, driveSpeed, -7, -7, 10);
        //4.66

        robot.encoderDrive(this, driveSpeed, 10, -10, 10);
        //6.66
            //Turning right towards beacon 2


        robot.encoderDrive(this, driveSpeed, 52, 52, 10);
        //34.66
            //Driving towards beacon 2


        robot.encoderDrive(this, driveSpeed, -12, 12, 10);
        //8
            //Turning left at Beacon 2


        robot.encoderDrive(this, driveSpeed, 15, 15, 10);
        //10
            //Driving Towards Beacon 2


        if (robot.color.red() > 0 && robot.color.red() > robot.color.blue()) {
            robot.leftPush.setPosition(robot.lpushDeploy);
        } else if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()){
            robot.rightPush.setPosition(robot.rpushDeploy);
        } else if (robot.color.red() == robot.color.blue()){
            telemetry.addData("ColorDecision: ", "Not Pressing");
            telemetry.update();
        }
        idle();
        sleep(1000);
        robot.leftPush.setPosition(robot.lpushStart);
        robot.rightPush.setPosition(robot.rpushStart);
        idle();

        robot.encoderDrive(this, driveSpeed, -11, -11, 10);
        //7.33
            //Backing up from Beacon 2


        robot.encoderDrive(this, driveSpeed, 5, -5, 10);
        //3.33
            //Turning towards Cap


        robot.encoderDrive(this, driveSpeed, -60, -60, 10);
        //40
            //Moving towards Cap ball



    }
}