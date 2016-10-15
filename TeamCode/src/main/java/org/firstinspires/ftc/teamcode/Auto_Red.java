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
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.AUTOTEST;

/**
 * Created by Devan on 10/9/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto_Red", group = "Sensor")
public class Auto_Red extends LinearOpMode{
    ColorSensor colorSensor;
    Servo leftPush;
    Servo rightPush;
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    double leftPushStart = 0.1;
    double rightPushStart = 0.1;
    double leftPushSecond = 0.9;
    double rightPushSecond = 0.9;
    HardwareHelper robot = new HardwareHelper(AUTOTEST);

    @Override
    public void runOpMode() throws InterruptedException {

//        waitForStart();
//
//     hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
//
//    // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
//
//    // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        colorSensor = hardwareMap.colorSensor.get("color sensor");
        leftPush = hardwareMap.servo.get("left_push");
        rightPush = hardwareMap.servo.get("right_push");
        leftMotorFront = hardwareMap.dcMotor.get("L Front");
        rightMotorFront = hardwareMap.dcMotor.get("R Front");

        while ( !isStarted() ) {
            telemetry.addData("Init:  %s", "Waiting for start");
            telemetry.update();
            idle();
        }

//        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
//
//      // send the info back to driver station using telemetry function.
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);


        robot.encoderDrive(this, .5, 72, 72, 10);
            leftMotorFront.setTargetPosition(8000 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(8000 + rightMotorFront.getCurrentPosition());
            telemetry.addData("Path1", "Running to %7d :%7d",
                    leftMotorFront.getTargetPosition(),
                    rightMotorFront.getTargetPosition());
            leftMotorFront.setPower(0.5);
            rightMotorFront.setPower(0.5);
            idle();

            //Turning at Beacon 1
            //Is there a way to do it in encoder ticks, to be more precise
            //How do I put a wait 1 second after the set power?
            leftMotorFront.setTargetPosition(-500 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(500 + rightMotorFront.getCurrentPosition());
            leftMotorFront.setPower(0.5);
            rightMotorFront.setPower(0.5);
            idle();
        
            //Driving Towards Beacon 1
            leftMotorFront.setTargetPosition(1976 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(1976 + rightMotorFront.getCurrentPosition());
            idle();

            // send color values info back to driver station
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            //Logic for pressing correct button
            //May need to be edited if we want to use the Hue values instead
            if (colorSensor.red() == 1) {
                leftPush.setPosition(leftPushSecond);
            } else {
                rightPush.setPosition(rightPushSecond);
            }
            wait(2000);
            leftPush.setPosition(leftPushStart);
            rightPush.setPosition(rightPushStart);
            idle();

            //Backing up from Beacon 1
            //Does the Encoder value need to be negative too?
            leftMotorFront.setTargetPosition(-728 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(-728 + rightMotorFront.getCurrentPosition());
            leftMotorFront.setPower(0.5);
            rightMotorFront.setPower(0.5);
            idle();

            //Turning right towards beacon 2
            //How do you put a wait, or how do you do this in encoder ticks?
            leftMotorFront.setTargetPosition(750 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(-750 + rightMotorFront.getCurrentPosition());
            leftMotorFront.setPower(0.5);
            rightMotorFront.setPower(0.5);

            //Driving towards beacon 2
            leftMotorFront.setTargetPosition(5408 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(5408 + rightMotorFront.getCurrentPosition());
            leftMotorFront.setPower(0.5);
            rightMotorFront.setPower(0.5);

            //Turning left at Beacon 2
            leftMotorFront.setTargetPosition(-750 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(750 + rightMotorFront.getCurrentPosition());
            leftMotorFront.setPower(0.5);
            rightMotorFront.setPower(0.5);

            //Driving Towards Beacon 2
            leftMotorFront.setTargetPosition(1976 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(1976 + rightMotorFront.getCurrentPosition());

            // send color values info back to driver station
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            //Logic for pressing correct button
            //May need to be edited if we want to use the Hue values instead
            if (colorSensor.red() == 1) {
                leftPush.setPosition(leftPushSecond);
            } else {
                rightPush.setPosition(rightPushSecond);
            }
            wait(2000);
            leftPush.setPosition(leftPushStart);
            rightPush.setPosition(rightPushStart);

            //Backing up from Beacon 2
            leftMotorFront.setTargetPosition(-1144 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(-1144 + rightMotorFront.getCurrentPosition());
            leftMotorFront.setPower(0.5);
            rightMotorFront.setPower(0.5);

            //Turning towards Cap ball
            leftMotorFront.setTargetPosition(500 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(-500 + rightMotorFront.getCurrentPosition());
            leftMotorFront.setPower(0.5);
            rightMotorFront.setPower(0.5);

            //Moving towards Cap ball
            leftMotorFront.setTargetPosition(-5200 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(-5200 + rightMotorFront.getCurrentPosition());
            leftMotorFront.setPower(0.5);
            rightMotorFront.setPower(0.5);

    }
}