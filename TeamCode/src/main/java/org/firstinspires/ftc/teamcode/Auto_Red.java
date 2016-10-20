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

/**
 * Created by Devan on 10/9/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto_Red", group = "Sensor")
public class Auto_Red extends LinearOpMode{

    HardwareHelper robot = new HardwareHelper(AUTOTEST);

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

        //colorSensor = hardwareMap.colorSensor.get("color sensor");
        //leftPush = hardwareMap.servo.get("left_push");
        //rightPush = hardwareMap.servo.get("right_push");
        //leftMotorFront = hardwareMap.dcMotor.get("L Front");
        //rightMotorFront = hardwareMap.dcMotor.get("R Front");
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



        robot.encoderDrive(this, .5, 77, 77, 10);
        //  leftMotorFront.setTargetPosition(8000 + leftMotorFront.getCurrentPosition());
        //  rightMotorFront.setTargetPosition(8000 + rightMotorFront.getCurrentPosition());
        //  telemetry.addData("Path1", "Running to %7d :%7d",
        //          leftMotorFront.getTargetPosition(),
        //          rightMotorFront.getTargetPosition());
        //  leftMotorFront.setPower(0.5);
        //  rightMotorFront.setPower(0.5);
        //  idle();

            //Turning at Beacon 1
            //Is there a way to do it in encoder ticks, to be more precise
            //How do I put a wait 1 second after the set power?

        robot.encoderDrive(this, .5, -10, 10, 5);

        // leftMotorFront.setTargetPosition(-500 + leftMotorFront.getCurrentPosition());
        //  rightMotorFront.setTargetPosition(500 + rightMotorFront.getCurrentPosition());
        //  leftMotorFront.setPower(0.5);
        //   rightMotorFront.setPower(0.5);
        //  idle();
        robot.encoderDrive(this, .5, 19, 19, 10);

        telemetry.addData("Auto: ", "Drive completed, looking for color sensor");
        telemetry.update();
        this.sleep(5000);
        telemetry.addData("Color:" , "Connection Info %s ", robot.color.getConnectionInfo());
        telemetry.update();

            //Logic for pressing correct button
            //May need to be edited if we want to use the Hue values instead
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
            if (robot.color.blue() == 1) {
                robot.leftPush.setPosition(robot.lpushDeploy);
            } else {
                robot.rightPush.setPosition(robot.rpushDeploy);
            }
            idle();
            sleep(2000);
            robot.leftPush.setPosition(robot.lpushStart);
            robot.rightPush.setPosition(robot.rpushStart);
            idle();

        robot.encoderDrive(this, .5, -7, -7, 10);
            //Backing up from Beacon 1
            //Does the Encoder value need to be negative too?
        //leftMotorFront.setTargetPosition(-728 + leftMotorFront.getCurrentPosition());
        //  rightMotorFront.setTargetPosition(-728 + rightMotorFront.getCurrentPosition());
        //  leftMotorFront.setPower(0.5);
        //  rightMotorFront.setPower(0.5);
        //  idle();
        robot.encoderDrive(this, .5, 7.2, -7.2, 10);
            //Turning right towards beacon 2
            //How do you put a wait, or how do you do this in encoder ticks?
        // leftMotorFront.setTargetPosition(750 + leftMotorFront.getCurrentPosition());
        //  rightMotorFront.setTargetPosition(-750 + rightMotorFront.getCurrentPosition());
        //  leftMotorFront.setPower(0.5);
        //  rightMotorFront.setPower(0.5);

        robot.encoderDrive(this, .5, 52, 52, 10);
            //Driving towards beacon 2
        //leftMotorFront.setTargetPosition(5408 + leftMotorFront.getCurrentPosition());
        //  rightMotorFront.setTargetPosition(5408 + rightMotorFront.getCurrentPosition());
        //  leftMotorFront.setPower(0.5);
        //  rightMotorFront.setPower(0.5);

        robot.encoderDrive(this, .5, -7.2, 7.2, 10);
            //Turning left at Beacon 2
        //leftMotorFront.setTargetPosition(-750 + leftMotorFront.getCurrentPosition());
        //  rightMotorFront.setTargetPosition(750 + rightMotorFront.getCurrentPosition());
        //  leftMotorFront.setPower(0.5);
        //  rightMotorFront.setPower(0.5);

        robot.encoderDrive(this, .5, 19, 19, 10);
            //Driving Towards Beacon 2
        //leftMotorFront.setTargetPosition(1976 + leftMotorFront.getCurrentPosition());
        //  rightMotorFront.setTargetPosition(1976 + rightMotorFront.getCurrentPosition());

            //Logic for pressing correct button
            //May need to be edited if we want to use the Hue values instead
        if (robot.color.red() == 1) {
            robot.leftPush.setPosition(robot.lpushDeploy);
        } else {
            robot.rightPush.setPosition(robot.rpushDeploy);
        }
        sleep(2000);
        robot.leftPush.setPosition(robot.lpushStart);
        robot.rightPush.setPosition(robot.rpushStart);
        idle();

        robot.encoderDrive(this, .5, -10.7, -10.7, 10);
            //Backing up from Beacon 2
        //leftMotorFront.setTargetPosition(-1144 + leftMotorFront.getCurrentPosition());
        //  rightMotorFront.setTargetPosition(-1144 + rightMotorFront.getCurrentPosition());
        //  leftMotorFront.setPower(0.5);
        //  rightMotorFront.setPower(0.5);

        robot.encoderDrive(this, .5, 4.8, -4.8, 10);
            //Turning towards Cap ball
            //leftMotorFront.setTargetPosition(500 + leftMotorFront.getCurrentPosition());
        //rightMotorFront.setTargetPosition(-500 + rightMotorFront.getCurrentPosition());
        //  leftMotorFront.setPower(0.5);
        //  rightMotorFront.setPower(0.5);

        robot.encoderDrive(this, .5, -50, 50, 10);
            //Moving towards Cap ball
        //  leftMotorFront.setTargetPosition(-5200 + leftMotorFront.getCurrentPosition());
        //  rightMotorFront.setTargetPosition(-5200 + rightMotorFront.getCurrentPosition());
        //  leftMotorFront.setPower(0.5);
        //  rightMotorFront.setPower(0.5);


    }
}