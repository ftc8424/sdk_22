package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLTELEOP;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.TROLLBOT;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.AUTOTEST;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.LAUNCHTEST;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.COLORTEST;

/**
 * Created by FTC8424 on 9/15/2016.
 */

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="DragRacer", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class DragRacer extends OpMode {


    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;


//    private Servo LPush;
//    double LPushStart = 0.1;
//    double LPushEnd = 0.9;
//    double LPushPower = 1.1;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        leftBackDrive = hardwareMap.dcMotor.get("L Back");
        rightBackDrive = hardwareMap.dcMotor.get("R Back");
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
//        LPush = hardwareMap.servo.get("left_push");
//        LPush.setPosition(LPushStart);
//        wait(2000);
//        LPush.setPosition(LPushEnd);


   /*
    * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    */


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        telemetry.addData("Status", "Running: " + runtime.toString());

        double rightStickVal = -gamepad1.right_stick_y;
        double leftStickVal = -gamepad1.left_stick_y;
        double rightSquaredVal = rightStickVal * rightStickVal;
        double leftSquaredVal = leftStickVal * leftStickVal;

        if (rightStickVal < 0) rightSquaredVal = -rightSquaredVal;
        if (leftStickVal < 0) leftSquaredVal = -leftSquaredVal;
        leftBackDrive.setPower(-leftSquaredVal);
        rightBackDrive.setPower(-rightSquaredVal);
    }
}