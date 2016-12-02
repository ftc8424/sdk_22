package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.AUTOTEST;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.COLORTEST;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLTELEOP;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.LAUNCHTEST;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.TROLLBOT;

/**
 * Created by gorpong on 10/13/2016.
 *
 * This is the initial helper class for the hardware componnets of E-Cubed (FTC8424) robot.
 * It is NOT an OpMode or any of the others, it's a helper class that has the hardware
 * map and some of the helper methods that are needed for all OpModes.
 *
 * This class assumes that the following hardware components have been given the following
 * names in the configuration file.  If they have not, then bad things will happen.
 *
 * Left Middle Drive Motor:    "L Mid"
 * Right Middle Drive Motor:   "R Mid"
 * Left Back Drive Motor:      "L Back"
 * Right Back Drive Motor:     "R Back"
 * Left Push Button Servo:     "L Push"  (servo)
 * Right Push Button Servo:    "R Push"
 * Color Sensor:               "color"
 * Launcher Motor:             "Launcher"
 * Lift for Launcher Servo:    "LaunchServo"
 *
 */

public class HardwareHelper {

    /* Public OpMode members, things they can use */
    public DcMotor  leftMidDrive = null;   private static final String cfgLMidDrive  = "L Mid";
    public DcMotor  rightMidDrive = null;  private static final String cfgRMidDrive  = "R Mid";
    public DcMotor  leftBackDrive = null;  private static final String cfgLBckDrive  = "L Back";
    public DcMotor  rightBackDrive = null; private static final String cfgRtBckDrive = "R Back";
    public DcMotor  launchMotor = null;    private static final String cfgLaunchMotor = "Launcher";
    public Servo    launchServo = null;    private static final String cfgLaunchServo = "LaunchServo";
    public Servo    leftPush = null;       private static final String cfgLPush       = "L Push";
    public Servo    rightPush = null;      private static final String cfgRPush       = "R Push";
    public ColorSensor color = null;       private static final String cfgColor       = "color";
    public DcMotor  manipMotor = null;    private static final String cfgmanipMotor = "Manipulator";

    /* Servo positions, adjust as necessary. */
    public static final double lpushStart = 0.7;
    public static final double lpushDeploy = 0;
    public static final double rpushStart = 0.3;
    public static final double rpushDeploy = 1;
    public static final double launchliftStart = .75;
    public static final double launchliftDeploy = 0.1;


    /* Use this when creating the constructor, to state the type of robot we're using. */
    public enum RobotType {
        FULLTELEOP, FULLAUTO, LAUNCHTEST, COLORTEST, AUTOTEST, TROLLBOT,
    }

    /* Private instance variables */

    /* Wheel ratio values for the encoders. */
    private static final double encoderInch  = 104; //2500.0 / (3 * 3.14169265);
    private static final double encoderRatio = 0.667; // / 1.33;    // 3" wheels so ratio is 1:1, 4 is 1/1.3

    private RobotType robotType;
    private HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public HardwareHelper(RobotType type) {
        robotType = type;
    }

    
    public void autoLauncher(LinearOpMode caller, double power) throws InterruptedException {

        launchMotor.setPower(power);
        caller.telemetry.addData("Motor", "LaunchPower Set to " + launchMotor.getCurrentPosition());

        caller.sleep(1750);
        if ( !caller.opModeIsActive() ) return;
        launchServo.setPosition(launchliftDeploy);
//        launchMotor.setPower(0.75);
        caller.sleep(500);
        if ( !caller.opModeIsActive() ) return;
        launchServo.setPosition(launchliftStart);
        caller.sleep(750);
        if ( !caller.opModeIsActive() ) return;
        launchServo.setPosition(launchliftDeploy);
        caller.sleep(500);
        if ( !caller.opModeIsActive() ) return;
        launchServo.setPosition(launchliftStart);
        launchMotor.setPower(0);
    }

    public void robot_init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        /* Set the drive motors in the map */
        if ( robotType == TROLLBOT || robotType == FULLTELEOP || robotType == FULLAUTO || robotType == AUTOTEST ) {
            leftBackDrive = hwMap.dcMotor.get(cfgLBckDrive);
            rightBackDrive = hwMap.dcMotor.get(cfgRtBckDrive);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            if ( robotType == FULLAUTO || robotType == FULLTELEOP ) {
                leftMidDrive = hwMap.dcMotor.get(cfgLMidDrive);
                rightMidDrive = hwMap.dcMotor.get(cfgRMidDrive);
                rightMidDrive.setDirection(DcMotor.Direction.REVERSE);

                if ( robotType == FULLAUTO ) {
                    leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftMidDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightMidDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }
        }

        /* Set the subsequent motors based on type */
        if ( robotType == LAUNCHTEST || robotType == FULLTELEOP || robotType == FULLAUTO ) {
            launchMotor = hwMap.dcMotor.get(cfgLaunchMotor);
            launchMotor.resetDeviceConfigurationForOpMode();
            launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launchMotor.setMaxSpeed(2000);
            launchMotor.setDirection(DcMotor.Direction.FORWARD);
            launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            manipMotor = hwMap.dcMotor.get(cfgmanipMotor);
            //manipMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        /* Set the servos based on type */
        if ( robotType == TROLLBOT || robotType == FULLTELEOP || robotType == FULLAUTO || robotType == AUTOTEST ) {
            leftPush = hwMap.servo.get(cfgLPush);
            rightPush = hwMap.servo.get(cfgRPush);
            rightPush.setPosition(rpushStart);
            leftPush.setPosition(lpushStart);
        }
        if ( robotType == LAUNCHTEST || robotType == FULLAUTO || robotType == FULLTELEOP ) {
            launchServo = hwMap.servo.get(cfgLaunchServo);
            launchServo.setPosition(launchliftStart);
        }

        /* Set the sensors based on type */
        if ( robotType == AUTOTEST || robotType == COLORTEST || robotType == FULLAUTO ) {
            color = hwMap.colorSensor.get(cfgColor);
        }

        /* Now that hardware is mapped, set to initial positions/settings. */
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        if ( robotType == FULLTELEOP || robotType == FULLAUTO ) {
            leftMidDrive.setPower(0);
            rightMidDrive.setPower(0);
        }
    }

    /**
     * Drive by the encoders, running to a position relative to the current position based
     * on encoder ticks for a left and right motor.  It will move to a position for a specified
     * period of time, so it will stop if they get to the desired position, if the time runs out
     * or if the OpMode is cancelled.
     *
     * Originally written in PushbotAutoDriveByEncoder_Linear.java from FtcRobotController area.
     *
     * @param caller                  Reference to calling class, must be LinearOpMode
     * @param speed                   The speed of the movement
     * @param leftInches              The target position of left motors, in inches from current
     * @param rightInches           The target position of right motors, in inches from current
     *
    * @param timeoutS                The timeout in seconds to allow the move
     * @throws InterruptedException
     */
    public void encoderDrive(LinearOpMode caller,
                             double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {

        int newLeftMidTarget;
        int newRightMidTarget;
        double timeOut = Math.abs(leftInches)/12 * 750;

        // Ensure that the opmode is still active
        if ( !caller.opModeIsActive() )
            return;

        if ( robotType == FULLAUTO ) {
//            leftMidDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightMidDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMidDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMidDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        // Determine new target position, and pass to motor controller

        newLeftMidTarget = leftMidDrive.getCurrentPosition() + (int)Math.round(leftInches * encoderInch * encoderRatio);
        newRightMidTarget = rightMidDrive.getCurrentPosition() + (int)Math.round(rightInches * encoderInch * encoderRatio);
        caller.telemetry.addData("encoderDrive: ", " Left Mid Target POS: %d / Right Mid Target POS:%d" ,  newLeftMidTarget, newRightMidTarget);
        caller.telemetry.update();

        leftMidDrive.setTargetPosition(newLeftMidTarget);
        rightMidDrive.setTargetPosition(newRightMidTarget);
        caller.telemetry.addData("Encoder Drive: ", "Target Set");

        // reset the timeout time and start motion.


//        if ( robotType == FULLAUTO ) {
//            leftMidDrive.setPower(Math.abs(speed));
//            rightMidDrive.setPower(Math.abs(speed));
//        }

        caller.idle();
        caller.telemetry.addData("Encoder Drive: ", "Power set to %.2f", Math.abs(speed));
        caller.telemetry.update();

        // keep looping while we are still active, and there is time left, and both motors are running.
        boolean isBusy;
        runtime.reset();
        do {
            rightBackDrive.setPower(rightInches < 0 ? -1 : 1 * Math.abs(speed));
            leftBackDrive.setPower(leftInches < 0 ? -1 : 1 * Math.abs(speed));
            rightMidDrive.setPower(rightInches < 0 ? -1 : 1 * Math.abs(speed));
            leftMidDrive.setPower(leftInches < 0 ? -1 : 1 * Math.abs(speed));
            //rightMidDrive.setPower(Math.abs(speed));
            //leftMidDrive.setPower(Math.abs(speed));
            caller.telemetry.addData("Drives", "Running to %7d : %7d",
                     newLeftMidTarget, newRightMidTarget);
            caller.telemetry.addData("Drives", "Currently at %7d : %7d",
                    leftMidDrive.getCurrentPosition(),
                    rightMidDrive.getCurrentPosition());
            caller.telemetry.update();

            // Allow time for other processes to run.
            caller.idle();
            //isBusy = (Math.abs(leftMidDrive.getCurrentPosition() - newLeftMidTarget) >= 5) && (Math.abs(rightMidDrive.getCurrentPosition() - newRightMidTarget) >= 5);
        }
        //while (caller.opModeIsActive() && (runtime.seconds() < timeoutS) && isBusy);
        while(runtime.milliseconds() < timeOut);
        // Stop all motion;
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        if ( robotType == FULLAUTO ) {
            leftMidDrive.setPower(0);
            rightMidDrive.setPower(0);
        }

        // Turn off RUN_TO_POSITION

        if ( robotType == FULLAUTO ) {
            leftMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    /**
     * Drive the robot forward/backward based on power settings passed in.
     *
     * @param leftPower      Power setting (-1.0 - 1.0)
     * @param rightPower     Power setting (-1.0 - 1.0)
     */

    public void normalDrive (OpMode caller, double leftPower, double rightPower) {
        leftBackDrive.setPower(leftPower);
        rightBackDrive.setPower(rightPower);
        if ( robotType == FULLTELEOP ) {
            leftMidDrive.setPower(leftPower);
            rightMidDrive.setPower(rightPower);
        }
        caller.telemetry.addData("normalDrive:", "Power set to L:%.2f, R:%.2f", leftPower, rightPower);
    }
}
