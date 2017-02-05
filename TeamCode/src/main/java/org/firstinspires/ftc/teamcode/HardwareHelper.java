package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
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
 * Created by FTC8424 on 10/13/2016.
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
    public DcMotor  leftMidDrive = null;   private static final String cfgLMidDrive   = "L Mid";
    public DcMotor  rightMidDrive = null;  private static final String cfgRMidDrive   = "R Mid";
    public DcMotor  leftBackDrive = null;  private static final String cfgLBckDrive   = "L Back";
    public DcMotor  rightBackDrive = null; private static final String cfgRtBckDrive  = "R Back";
    public DcMotor  launchMotor = null;    private static final String cfgLaunchMotor = "Launcher";
    //public DcMotor  launchMotor2 = null;    private static final String cfgLaunchMotor2 = "Launcher2";
    public Servo    launchServo = null;    private static final String cfgLaunchServo = "LaunchServo";
    public Servo    leftPush = null;       private static final String cfgLPush       = "L Push";
    public Servo    rightPush = null;      private static final String cfgRPush       = "R Push";
    public ColorSensor color = null;       private static final String cfgColor       = "color";
    public DcMotor  manipMotor = null;     private static final  String cfgmanipMotor = "Manipulator";
    public ModernRoboticsI2cGyro gyro = null;    private static final      String cfgGyro        = "gyro";

    /* Servo positions, adjust as necessary. */
    public static final double lpushStart = 1.0;
    public static final double lpushDeploy = 0.6;
    public static final double rpushStart = 0.4;
    public static final double rpushDeploy = 1;
    public static final double launchliftStart = .80;
    public static final double launchliftDeploy = 0.1;
    private static final int Samplesize = 250;
    private int[] encTicks = new int[Samplesize];
    private double[] encTime = new double [Samplesize];
    private int encIndex = 0;
    private double launchStarted = 0;
    private int launchEncoderStart = 0;
    private boolean isLauncherRunning = false;



    /* Use this when creating the constructor, to state the type of robot we're using. */
    public enum RobotType {
        FULLTELEOP, FULLAUTO, LAUNCHTEST, COLORTEST, AUTOTEST, TROLLBOT,
    }

    /*
     * Private instance variables
     */

    /* Wheel ratio values for the encoders (see end of this file for calculations). */
    private static final int   COUNTS_PER_SECOND_MAX = 2800;  // AndyMark NeveRest 40:1/20:1
    private static final double COUNTS_PER_MOTOR_REV = 1120;  // AndyMark NeveRest 40:1 CPR
    private static final double DRIVE_GEAR_REDUCTION = 1.0;   // No gears, just motor shafts
    private static final double WHEEL_DIAMETER_INCHES= 4.0;   // 4" Omni wheels and 4" Stealth
    private static final double encoderInch = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                   (WHEEL_DIAMETER_INCHES * 3.14159265);
    private static final int    COUNTS_PER_LAUNCHER  = 1060;  // AndyMark NeveRest 20:1, ideal

    /* Other privates for things such as the runtime, the hardware Map, etc. */
    private RobotType robotType;
    private HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();



    /**
     * Constructor for HardwareHelper, pass in the enumerated type RobotType based on the type of
     * OpMode we are running (e.g., a Trollbot; TeleOp for the full robot with all sensors, servos,
     * motors, etc.; Testing autonomous with only partial motor configurations, etc.  This is then
     * used throughout the rest of the HardwareHelper methods to make them dynamically figure out
     * what to do (e.g., if encoderDrive() is called and we have four motors based on the
     * enumerated type, then send power to all motors, otherwise just those that are on the
     * minimum robot).
     *
     * @param type    The enumerated type of the robot
     */
    public HardwareHelper(RobotType type) {
        robotType = type;
    }

    /**
     * This is the initialization routine for every OpMode in the system.  It uses the RobotType
     * as passed in the contrustor to determine which elements of the physical robot to go
     * and get from the hardware map.  It also expects to be sent the hardware map that is used
     * by the specific OpMode (not sure if there is only a single HardwareMap per the FTC
     * Robot Controller, or if each OpMode has its own, so just pass in whatever your OpMode
     * has and robot_init() will take care of it).
     *
     * This method goes and instantiates every element in the hardware map that is appropriate
     * for the type of robot we are and then sets the initial configuration options for them.
     *
     * @param hwMap    The hardware map entry from the OpMode.
     */
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
            }

            /*
             * If autonomous, then reset encoders and set mode to be with encoders
                    * NOTE:  This should really throw an exception or something, but all it does
                    * is silently ignore if the resetting of the encoders didn't work and blindly
                    * sets the mode to be RUN_USING_ENCODER.  Can't really call telemetry because
                    * don't have a caller to know which should build the telemetry and send.  Badness
                    * will ensue when the actual autonomous runs and hopefully this note will help
                    * folks figure out the failed reset might be at fault.
                    */
            boolean resetOk = false;
            if ( robotType == AUTOTEST || robotType == FULLAUTO ) {
                resetOk = waitForReset(leftBackDrive, rightBackDrive, 2000);
                if ( robotType == FULLAUTO )
                    resetOk = resetOk && waitForReset(leftMidDrive, rightMidDrive, 2000);
                if (resetOk) {
                    leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if ( robotType == FULLAUTO ) {
                        leftMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rightMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }
            }
        }

        /* Set the subsequent motors based on type */
        if ( robotType == LAUNCHTEST || robotType == FULLTELEOP || robotType == FULLAUTO ) {
            launchMotor = hwMap.dcMotor.get(cfgLaunchMotor);
            //vc1`launchMotor2 = hwMap.dcMotor.get(cfgLaunchMotor2);
            launchMotor.resetDeviceConfigurationForOpMode();
            launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launchMotor.setMaxSpeed(COUNTS_PER_SECOND_MAX);
            launchMotor.setDirection(DcMotor.Direction.FORWARD);
            launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            manipMotor = hwMap.dcMotor.get(cfgmanipMotor);
            //manipMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        /* Set the servos based on type */
        if (  robotType == FULLTELEOP || robotType == FULLAUTO || robotType == AUTOTEST ) {
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

        /* Get the Gyro */
        if (robotType == FULLAUTO ) {
            gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get(cfgGyro);
        }

        /* Now that hardware is mapped, set to initial positions/settings. */
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        if ( robotType == FULLTELEOP || robotType == FULLAUTO ) {
            leftMidDrive.setPower(0);
            rightMidDrive.setPower(0);

        }
        isLauncherRunning = false;
        initLaunchArray();
        prevEncoderSaved = 0;
        prevTimeSaved = 0;
    }


    /**
     * Drive by the encoders, running to a position relative to the current position based
     * on encoder ticks for a left and right motor.  It will move to a position for a specified
     * period of time, so it will stop if they get to the desired position, if the time runs out
     * or if the OpMode is cancelled.
     *
     * Originally written in PushbotAutoDriveByEncoder_Linear.java from FtcRobotController area,
     * modified by FTC8424 for defensive encoder moves.
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
        int newLeftBackTarget;
        int newRightBackTarget;
        long encoderTimeout = 2000;   // Wait no more than two seconds, an eternity, to set

        if ( !caller.opModeIsActive() )
            return;



        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if ( robotType == FULLAUTO ) {
            leftMidDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMidDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        /*
         * Determine new target position and pass to motor controller
         */
        if ( robotType == FULLAUTO ) {
            newLeftMidTarget = leftMidDrive.getCurrentPosition() + (int) Math.round(leftInches * encoderInch);
            newRightMidTarget = rightMidDrive.getCurrentPosition() + (int) Math.round(rightInches * encoderInch);
        } else {
            newLeftMidTarget = 0;
            newRightMidTarget = 0;
        }
        newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)Math.round(leftInches * encoderInch);
        newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)Math.round(rightInches * encoderInch);
//        caller.telemetry.addLine("encoderDrive-MID:")
//                .addData("Left Tgt POS: ", newLeftMidTarget)
//                .addData("Right Tgt POS:" ,  newRightMidTarget);
//        caller.telemetry.addLine("EncoderDrive-BCK:")
//                .addData("Left Tgt POS: ", newLeftBackTarget)
//                .addData("Right Tgt POS: ", newRightBackTarget);
//        caller.telemetry.update();

        boolean lmEncoderSet = false;
        boolean rmEncoderSet = false;
        boolean lbEncoderSet = false;
        boolean rbEncoderSet = false;

        lbEncoderSet = setEncoderPosition(caller, leftBackDrive, newLeftBackTarget, encoderTimeout);
        rbEncoderSet = setEncoderPosition(caller, rightBackDrive, newRightBackTarget, encoderTimeout);
        if ( robotType == FULLAUTO ) {
            lmEncoderSet = setEncoderPosition(caller, leftMidDrive, newLeftMidTarget, encoderTimeout);
            rmEncoderSet = setEncoderPosition(caller, rightMidDrive, newRightMidTarget, encoderTimeout);
        } else {
            lmEncoderSet = true;
            rmEncoderSet = true;
        }
//        caller.telemetry.addLine("EncoderSet:")
//                .addData("LB: ", lbEncoderSet)
//                .addData("RB: ", rbEncoderSet)
//                .addData("LM: ", lmEncoderSet)
//                .addData("RM: ", rmEncoderSet);
//        caller.telemetry.update();
        if ( ! (lmEncoderSet && lbEncoderSet && rmEncoderSet && rbEncoderSet) ) {
            caller.telemetry.addLine("Encoders CANNOT be set, aborting OpMode");
            caller.telemetry.update();
            caller.sleep(10000);    // Can't go any further, allow telemetry to show, then return
            return;
        }

        // reset the timeout time and start motion.

//        caller.telemetry.addLine("Encoder Drive: ")
//                .addData("PowerSet: ", "%.4f", Math.abs(speed));
//        caller.telemetry.update();

        // keep looping while we are still active, and there is time left, and motors haven't made position.
        boolean isBusy;
        int lmCurPos;
        int rmCurPos;
        int lbCurPos;
        int rbCurPos;
        double stopTime = runtime.seconds() + timeoutS;
        do {
            rightBackDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            if ( robotType == FULLAUTO ) {
                rightMidDrive.setPower(Math.abs(speed));
                leftMidDrive.setPower(Math.abs(speed));
            }
//            caller.telemetry.addLine("Drives-TO ")
//                    .addData("POS ", "%7d : %7d : %7d : %7d",
//                            newLeftMidTarget, newRightMidTarget,
//                            newLeftBackTarget, newRightBackTarget);
//            caller.telemetry.update();
            lbCurPos = leftBackDrive.getCurrentPosition();
            rbCurPos = rightBackDrive.getCurrentPosition();
            if ( robotType == FULLAUTO ) {
                lmCurPos = leftMidDrive.getCurrentPosition();
                rmCurPos = rightMidDrive.getCurrentPosition();
            } else {
                lmCurPos = Integer.MAX_VALUE;
                rmCurPos = Integer.MAX_VALUE;
            }
//            caller.telemetry.addLine("Drives-CUR ")
//                    .addData("POS ", "%7d : %7d : %7d : %7d",
//                            lmCurPos, rmCurPos, lbCurPos, rbCurPos);
//            caller.telemetry.update();
            isBusy = (Math.abs(lbCurPos - newLeftBackTarget) >= 5) && (Math.abs(rbCurPos - newRightBackTarget) >= 5);
            if ( robotType == FULLAUTO )
                isBusy = isBusy && (Math.abs(lmCurPos - newLeftMidTarget) >= 5) && (Math.abs(rmCurPos - newRightMidTarget) >= 5);
        }
        while (caller.opModeIsActive() && isBusy && runtime.seconds() < stopTime);

        // Stop all motion;
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        if ( robotType == FULLAUTO ) {
            leftMidDrive.setPower(0);
            rightMidDrive.setPower(0);
        }

        // Turn off RUN_TO_POSITION

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if ( robotType == FULLAUTO ) {
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

    public void normalDrive (OpMode caller, double leftPower, double rightPower) {
        leftBackDrive.setPower(leftPower);
        rightBackDrive.setPower(rightPower);
        if ( robotType == FULLTELEOP ) {
            leftMidDrive.setPower(leftPower);
            rightMidDrive.setPower(rightPower);
        }
        caller.telemetry.addData("normalDrive:", "Power set to L:%.2f, R:%.2f", leftBackDrive.getPower(), rightBackDrive.getPower());
    }

    /**
     * Get the voltage of the 12V battery and return it.
     *
     * @return  The voltage of the 12V
     */
    public double getVoltage() {
        return hwMap.voltageSensor.iterator().next().getVoltage();
    }

    /*
     * Variables that are only used in the launcher stabilization methods.
     */
    private int prevEncoderSaved = 0;
    private double prevTimeSaved = 0;

    /**
     * Stop the launcher, and reset any variables as part of the launcher
     */
    public void stopLauncher() {
        launchMotor.setPower(0.0);
        isLauncherRunning = false;
    }

    /**
     * Start the launch motor and reset the variables for the automatic launch stabilization.
     * Grab the voltage of the battery and return that voltage to the caller.
     *
     * @return  the current voltage of the 12V main battery
     */
    public double startLauncher() {
        double batteryVoltage = getVoltage();

        // Clear out the timing and encoder ticks arrays
        initLaunchArray();
        launchMotor.setPower(0.65);
        isLauncherRunning = true;
        return batteryVoltage;
    }

    public double startLauncher(double power) {
        double batteryVoltage = getVoltage();

        // Clear out the timing and encoder ticks arrays
        initLaunchArray();
        launchMotor.setPower(power);
        isLauncherRunning = true;
        return batteryVoltage;
    }

    /**
     * Run the launcher to shoot two balls in Autonomous.  Written by Kim Tan and Mohana Chavan.
     * It launches the motor, allows 1.75 seconds to get up to speed, shoots a ball, allows another
     * 1.25 seconds to get back up to speed and launches the second.  Waits a half-second and
     * then shuts everything down and returns.
     *
     * It uses startLauncher() so that the power that the launch motor will be running at is
     * automatically set based on the voltage of the main battery.  It also automatically adjusts
     * the wait time before launching based on the voltage of the battery.
     *
     * @param caller  he "this" from caller, so I can run telemetry
     * @throws InterruptedException     If the sleep call is interrupted
     */
    public void autoLauncher(LinearOpMode caller, double power) throws InterruptedException {

        if ( !caller.opModeIsActive() ) return;


        double volts = startLauncher(power);
//        caller.telemetry.addData("Launch Motor", "Bat Voltage is %.2f / Power at %.2f",
//                volts, launchMotor.getPower());
//        caller.telemetry.update();
        while (caller.opModeIsActive() &&  ! adjustLaunchSpeed(caller)) {

            caller.telemetry.addData("Launch Motor", " Power at %.2f", launchMotor.getPower());
            caller.telemetry.update();

        }
            ;
        long sleepTime;
        if ( volts < 12.4 )
               sleepTime = 2500;
        else if ( volts < 12.6 )
                sleepTime = 2000;
        else if ( volts < 12.8 )
                sleepTime = 1900;
        else
            sleepTime = 1750;
        caller.telemetry.addData("Launch Motor", "Waiting %.2f Secs to spin-up", sleepTime / 1000.0);
        caller.telemetry.update();
        caller.sleep(sleepTime);
        caller.telemetry.addData("Launch Motor", "Bat Voltage is %.2f / Power at %.2f",
                volts, launchMotor.getPower());
        caller.telemetry.update();
        if ( !caller.opModeIsActive() ) return;
        launchServo.setPosition(launchliftDeploy);      // Shoot the first ball
        initLaunchArray();
        caller.sleep(500);
        //double stopIn = runtime.milliseconds() + 500;   // Stop in half a second
        // allow launcher to stabilize again



        //caller.sleep(500);
        launchServo.setPosition(launchliftStart);
        launchMotor.setPower(launchMotor.getPower() + 0.1);
        caller.sleep(2200);
        /*
         * Before shoot the second, let the power get back up to speed, should be fast
         */
        while (caller.opModeIsActive() &&  ! adjustLaunchSpeed(caller)) {
            caller.telemetry.addData("Launch Motor", " Power at %.2f", launchMotor.getPower());
            caller.telemetry.update();
        }
        if ( !caller.opModeIsActive() ) return;
        launchServo.setPosition(launchliftDeploy);
        caller.sleep(500);
        if ( !caller.opModeIsActive() ) return;
        launchServo.setPosition(launchliftStart);
        stopLauncher();
    }

    /**
     * Initialize the variables used in the automatic Launch motor stabilization code.  This should
     * be run at any time we start the launch motor *and* any time that the power to the motor is
     * adjusted, so that when we check to see if it's stabilized we're dealing with fresh buckets
     * each time.
     */
    private void initLaunchArray() {
        for (int i = 0; i < Samplesize; i++) {
            encTicks[i] = 0;
            encTime[i] = 0;
        }
        encIndex = 0;
        prevEncoderSaved = launchMotor.getCurrentPosition();
        prevTimeSaved = runtime.milliseconds();
    }

    /**
     * Get the ticks from the launch encoder and save those in the next available bucket, if
     * necessary.  This is a key component in the automatic launch motor stabilization procedures.
     * It won't save anything if the value for the encoder ticks hasn't changed, which happens
     * in a regular OpMode about once in every three tries.
     *
     * It saves the delta in encoder ticks and time from the previous time getTicks() was
     * successfully called and returns whether it saved a value or not.
     *
     * @return    true if a value was saved in the bucket, false otherwise
     */
    private boolean getTicks() {
        if ( ! isLauncherRunning ) return false;
        int curEncoder = Math.abs(launchMotor.getCurrentPosition());
        double  curTime = runtime.milliseconds();

        if (prevEncoderSaved - curEncoder == 0 )
            return false;

        encTime[encIndex] = curTime - prevTimeSaved;
        encTicks[encIndex] = Math.abs(curEncoder - prevEncoderSaved);
        prevEncoderSaved = curEncoder;
        prevTimeSaved = curTime;
        encIndex = (encIndex == Samplesize-1) ? 0 : encIndex + 1;
        return true;
    }

   /**
     * Adjust the speed of the launch motor to get what we want out of the launcher before
     * we start launching.  We will return true if we are within our tolerance of where we
     * want to be, which is approximately 1,060 encoder ticks per second.
     *
     * @return  true if we're at optimum power, false if we're not (and adjusting)
     * @throws InterruptedException
     */
    public boolean adjustLaunchSpeed(OpMode caller) throws InterruptedException {
        return true;
//        if (!getTicks())
//            return false;
//        int totalTicks = 0;
//        double totalTime  = 0;
//        int count = 0;
//        for (int i = 0; i < Samplesize; i++) {
//            totalTicks = totalTicks + encTicks[i];
//            totalTime  = totalTime  + encTime[i];
//            if ( encTicks[i] > 0 ) count++;
//        }
//
//        int last = (encIndex == 0) ? Samplesize-1 : encIndex - 1;
//        int TicksAvg = totalTicks / count;
//        long TimeAvg = Math.round(totalTime / count);
//        double timeInSecs = TimeAvg / 1000.0;
//        long ticksInSecs = Math.round(TicksAvg / timeInSecs);
//        caller.telemetry.addData("adjustLaunchSpeed", "TicksAvg %d, TimeAvg %d, timeInSecs %.2f, ticksInSecs: %d, Count: %d",
//                TicksAvg, TimeAvg, timeInSecs, ticksInSecs, count);
//        caller.telemetry.update();
//
//        if ( count <= (Samplesize / 2) ) return false;    // Not large enough, keep going.
//
//        if (Math.abs(encTicks[0] - TicksAvg) <= 100) {
//            if (Math.abs(ticksInSecs - COUNTS_PER_LAUNCHER) <= 100) {
//                return true;
//            } else if (ticksInSecs > COUNTS_PER_LAUNCHER && launchMotor.getPower() > 0.1) {
//                launchMotor.setPower(launchMotor.getPower() - 0.05);
//                initLaunchArray();    // Clear out buckets, so we can see if new power is right
//                return false;
//            } else {
//                launchMotor.setPower(launchMotor.getPower() + 0.05);
//                initLaunchArray();    // Clear out buckets, so we can see if new power is right
//                return false;
//            }
//        } else {
//            return false;
//        }
    }

        /**
         * Waits for the encoders to be reset on the 4 motors, and returns a boolean as to whether
         * the motors have reset or not.  Calls waitForReset() with two motor method signature multiple
         * times to do the actual work and combines the return values to give an overall return of
         * true only if all four motors were properly reset.
         *
         * @param m1    Motor 1 to reset
         * @param m2    Motor 2 to reset
         * @param m3    Motor 3 to reset
         * @param m4    Motor 4 to reset
         * @param msTimeOut The time to wait, in milliseonds, for a valid reset
         * @return      Whether the reset was successful or not
         */
    public boolean waitForReset(DcMotor m1, DcMotor m2, DcMotor m3, DcMotor m4, long msTimeOut) {
        boolean resetOk = false;

        resetOk = waitForReset(m1, m2, msTimeOut/2);
        return resetOk && waitForReset(m3, m4, msTimeOut/2);
    }

    /**
     * Waits for the encoders to be reset on the 2 motors and returns a boolean as to whether
     * the motors have reset or not.  If they haven't reset in the timeOut milliseconds, then
     * it will return false.  This is the true method that does stuff, the other method with the
     * 4 motor signature just calls this method multiple times.
     *
     * @param m1        Motor 1 to reset
     * @param m2        Motor 2 to reset
     * @param msTimeOut   The time to wait, in milliseconds, for a valid reset
     * @return          Whether the reset was successful or not
     */
    public boolean waitForReset(DcMotor m1, DcMotor m2, long msTimeOut) {
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int m1Pos = m1.getCurrentPosition();
        int m2Pos = m2.getCurrentPosition();
        double stopTime = runtime.milliseconds() + msTimeOut;
        while ( (m1Pos != 0 || m2Pos != 0) && runtime.milliseconds() < stopTime ) {
            m1Pos = m1.getCurrentPosition();
            m2Pos = m2.getCurrentPosition();
        }
        return m1Pos == 0 && m2Pos == 0;
    }

    /****************************************************************************************
     * Private methods follow.
     ****************************************************************************************/

    /**
     * Sets the target encoder position for this particular motor and waits to ensure that the
     * position was properly set.  If it didn't set correctly, within the timeOut (in milliseconds)
     * time, then it will returns false.
     *
     * @param caller    The calling OpMode (for opModeIsActive() call)
     * @param m1        The motor on which to set the position
     * @param target    The target position to set
     * @param timeOut   The time, in milliseconds, for it to properly set
     * @return          True if target set within timeOut milliseconds, false otherwise
     */
    private boolean setEncoderPosition (LinearOpMode caller, DcMotor m1, int target, long timeOut) {
        m1.setTargetPosition(target);
        int m1Pos = m1.getTargetPosition();
        double stopTime = runtime.milliseconds() + timeOut;
        while ( caller.opModeIsActive() && m1Pos != target && runtime.milliseconds() < stopTime ) {
            m1.setTargetPosition(target);
            m1Pos = m1.getTargetPosition();
        }
        return m1Pos == target;
    }
}

/************************************************************************************************
 * For encoder math, here is the information from AndyMark's web site, so it will be key in
 * setting up the setMaxSpeed() when in PID mode, as well as when figuring out the counts per
 * inch mode.
 *
 *    NeveRest 40:1 Motors:
 *    ---------------------
 *          7 pulses per revolution of hall effect encoder, and a 40:1 gearbox, so 7*40 ==
 *        280 pulses per revolution of the encoder, there are 4 revolutions of encoder to output
 *       1120 pulses per revolution of the OUTPUT SHAFT (e.g., the motor shaft)
 *        150 revolutions per minute of output shaft (RPM), so (1120 * 150) / 60 ==
 *       2800 pulses per second is the max Speed setting of the encoders on this motor
 *
 *    NeveRest 20:1 Motors:
 *    ----------------------
 *          7 pulses per revolution of hall effect encoder, and a 20:1 gearbox, so 7*20 ==
 *        140 pulses per revolution of the encoder, there are 4 revolutions of encoder to output
 *        560 pulses per revolution of the OUTPUT SHAFT (e.g., the motor shaft)
 *        300 revolutions per minute of output shaft (RPM), so (560 * 300) / 60 ==
 *       2800 pulses per second is the max Speed setting of the encoders on this motor
 *
 * So, for these two motors, the encoder COUNTS_PER_MOTOR_REV above would be 1,120 for the 40:1
 * and 560 for the 20:1, and the COUNTS_PER_SECOND_MAX above would be 2800 for both.
 *
 *************************************************************************************************/
