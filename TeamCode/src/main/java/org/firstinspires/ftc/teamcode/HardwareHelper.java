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
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLROBOT;
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

    /* Wheel ratio values for the encoders. */
    private static final double encoderInch  = 104; //2500.0 / (3 * 3.14169265);
    private static final double encoderRatio = 1.0;    // 3" wheels so ratio is 1:1

    /* Servo positions, adjust as necessary. */
    public static final double lpushStart = 0.25;
    public static final double lpushDeploy = 1;
    public static final double rpushStart = 0.25;
    public static final double rpushDeploy = 1;
    public static final double launchliftStart = 0.1;
    public static final double launchliftDeploy = 0.25;

    /* Use this when creating the constructor, to state the type of robot we're using. */
    public enum RobotType {
        FULLROBOT, LAUNCHTEST, COLORTEST, AUTOTEST, TROLLBOT,
    }

    /* Private instance variables */
    private RobotType robotType;
    private HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public HardwareHelper(RobotType type) {
        robotType = type;
    }

    public void robot_init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        /* Set the drive motors in the map */
        if ( robotType == TROLLBOT || robotType == FULLROBOT || robotType == AUTOTEST ) {
            leftBackDrive = hwMap.dcMotor.get(cfgLBckDrive);
            rightBackDrive = hwMap.dcMotor.get(cfgRtBckDrive);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if ( robotType == FULLROBOT ) {
                leftMidDrive = hwMap.dcMotor.get(cfgLMidDrive);
                rightMidDrive = hwMap.dcMotor.get(cfgRMidDrive);
                manipMotor = hwMap.dcMotor.get(cfgmanipMotor);
                rightMidDrive.setDirection(DcMotor.Direction.REVERSE);
                manipMotor.setDirection(DcMotor.Direction.REVERSE);
                leftMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftMidDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMidDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }

        /* Set the subsequent motors based on type */
        if ( robotType == LAUNCHTEST || robotType == FULLROBOT  ) {
            launchMotor = hwMap.dcMotor.get(cfgLaunchMotor);
        }

        /* Set the servos based on type */
        if ( robotType == TROLLBOT || robotType == FULLROBOT || robotType == AUTOTEST ) {
            leftPush = hwMap.servo.get(cfgLPush);
            rightPush = hwMap.servo.get(cfgRPush);
            rightPush.setPosition(rpushStart);
            leftPush.setPosition(lpushStart);
        }
        if ( robotType == LAUNCHTEST || robotType == FULLROBOT ) {
            launchServo = hwMap.servo.get(cfgLaunchServo);
        }

        /* Set the sensors based on type */
        if ( robotType == AUTOTEST || robotType == COLORTEST || robotType == FULLROBOT ) {
            color = hwMap.colorSensor.get(cfgColor);
        }

        /* Now that hardware is mapped, set to initial positions/settings. */
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        if ( robotType == FULLROBOT ) {
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
     * @param rightInches             The target position of right motors, in inches from current
     * @param timeoutS                The timeout in seconds to allow the move
     * @throws InterruptedException
     */
    public void encoderDrive(LinearOpMode caller,
                             double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if ( !caller.opModeIsActive() )
            return;

        if ( robotType == FULLROBOT  ) {
            leftMidDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMidDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftBackDrive.getCurrentPosition() + (int)Math.round(leftInches * encoderInch * encoderRatio);
        newRightTarget = rightBackDrive.getCurrentPosition() + (int)Math.round(rightInches * encoderInch * encoderRatio);
        caller.telemetry.addData("encoderDrive: ", "Left Target POS:  %d / Right Target POS:  %d", newLeftTarget, newRightTarget);
        caller.telemetry.update();
        leftBackDrive.setTargetPosition(newLeftTarget);
        rightBackDrive.setTargetPosition(newRightTarget);
        if ( robotType == FULLROBOT ) {
            leftMidDrive.setTargetPosition(newLeftTarget);
            rightMidDrive.setTargetPosition(newRightTarget);
        }
    caller.telemetry.addData("Encoder Drive: ", "Target Set");
        // Turn On RUN_TO_POSITION
       // if ( robotType == FULLROBOT || robotType == AUTOTEST ) {
         //   leftMidDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           // rightMidDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //} else {
          //  leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       // }

        // reset the timeout time and start motion.
        runtime.reset();
        rightBackDrive.setPower(Math.abs(speed));
        leftBackDrive.setPower(Math.abs(speed));

        if ( robotType == FULLROBOT ) {
            leftMidDrive.setPower(Math.abs(speed));
            rightMidDrive.setPower(Math.abs(speed));
        }
        caller.idle();
        caller.telemetry.addData("Encoder Drive: ", "Power set to %.2f", Math.abs(speed));
        caller.telemetry.update();
        // keep looping while we are still active, and there is time left, and both motors are running.
        DcMotor leftDrive;
        DcMotor rightDrive;
        if ( robotType == FULLROBOT ) {
            leftDrive = leftMidDrive;
            rightDrive = rightMidDrive;
        } else {
            leftDrive = leftBackDrive;
            rightDrive = rightBackDrive;
        }
        while (caller.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                caller.telemetry.addData("Drives",  "Running to %7d :%7d",
                        newLeftTarget,  newRightTarget);
                caller.telemetry.addData("Drives",  "Currently at %7d :%7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                caller.telemetry.update();

                // Allow time for other processes to run.
                caller.idle();
        }

        // Stop all motion;
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        if ( robotType == FULLROBOT ) {
            leftMidDrive.setPower(0);
            rightMidDrive.setPower(0);
        }

        // Turn off RUN_TO_POSITION
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if ( robotType == FULLROBOT ) {
            leftMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    /**
     * Drive the robot forward/backward based on power settings passed in.
     *
     * @param leftPower      Power setting (-1.0 - 1.0)
     * @param rightPower     Power setting (-1.0 - 1.0)
     */

    public void normalDrive (double leftPower, double rightPower) {
        leftBackDrive.setPower(leftPower);
        rightBackDrive.setPower(rightPower);
        if ( robotType == FULLROBOT ) {
            leftMidDrive.setPower(leftPower);
            rightMidDrive.setPower(rightPower);
        }
    }
}

