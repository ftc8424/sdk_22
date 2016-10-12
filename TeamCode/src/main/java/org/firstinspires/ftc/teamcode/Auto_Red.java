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
/**
 * Created by Devan on 10/9/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Sensor: HT color", group = "Sensor")
public class Auto_Red extends LinearOpMode{
    ColorSensor colorSensor;
    Servo leftPush;
    Servo rightPush;
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
}

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
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

        waitForStart();

        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
//
//      // send the info back to driver station using telemetry function.
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);


        if (gamepad1.a) {
            leftMotorFront.setTargetPosition(8000 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(8000 + rightMotorFront.getCurrentPosition());
            telemetry.addData("Path1", "Running to %7d :%7d",
                    leftMotorFront.getTargetPosition(),
                    rightMotorFront.getTargetPosition());
            leftMotorFront.setPower(0.5);
            rightMotorFront.setPower(0.5);


            //Turning at Beacon 1
            //Is there a way to do it in encoder ticks, to be more precise
            //How do I put a wait 1 second after the set power?
            leftMotorFront.setPower(-0.5);
            rightMotorFront.setPower(0.5);

            //Driving Towards Beacon 1
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
                leftPush.setPosition(0.9);
            } else {
                rightPush.setPosition(0.9);
            }

            //Backing up from Beacon 1
            //Does the Encoder value need to be negative too?
            leftMotorFront.setTargetPosition(-728 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(-728 + rightMotorFront.getCurrentPosition());
            leftMotorFront.setPower(-0.5);
            rightMotorFront.setPower(-0.5);

            //Turning right towards beacon 2
            //How do you put a wait, or how do you do this in encoder ticks?
            leftMotorFront.setPower(0.5);
            rightMotorFront.setPower(-0.5);

            //Driving towards beacon 2
            leftMotorFront.setTargetPosition(5408 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(5408 + rightMotorFront.getCurrentPosition());
            leftMotorFront.setPower(0.5);
            rightMotorFront.setPower(0.5);

            //Turning at Beacon 2
            leftMotorFront.setPower(-0.5);
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
                leftPush.setPosition(0.9);
            } else {
                rightPush.setPosition(0.9);
            }

            //Backing up from Beacon 2
            leftMotorFront.setTargetPosition(-1144 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(-1144 + rightMotorFront.getCurrentPosition());

            //Turning towards Cap ball
            leftMotorFront.setPower(-0.5);
            rightMotorFront.setPower(0.5);

            //Moving towards Cap ball
            leftMotorFront.setTargetPosition(-5200 + leftMotorFront.getCurrentPosition());
            rightMotorFront.setTargetPosition(-5200 + rightMotorFront.getCurrentPosition());

        }
    }
}