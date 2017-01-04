package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLTELEOP;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.LAUNCHTEST;

/**
 * Created by FTC8424 on 12/10/2016.
 */



/**
 * Created by avc on 10/25/2016.
 */

@TeleOp(name="Speed_Test", group="Tests")

public class LauncherSpeedTest extends OpMode {

    double lastStateChange;
    int launcherState;
    private ElapsedTime runtime = new ElapsedTime();
    private HardwareHelper robot = new HardwareHelper(FULLTELEOP);
    private double LservoUpTime = 0;
    private double RservoUpTime = 0;
    private double powerSetTime = 0;
    private double servoUpTime = 0;
    private double launchPress = 0;
    private double decreaseSpeed = 0;
    private double PrevEncoderTime = 0;
    private int PrevEncoderValue = 0;

    @Override
    public void init() {
        robot.robot_init(hardwareMap);
        //robot.launchMotor.setMaxSpeed(3000);
    }

    @Override
    public void loop() {
        double voltage = 0;

        if (gamepad2.a && (launchPress + 2) < runtime.seconds()) {
            if ( robot.launchMotor.getPower() > 0.0 ) {
                robot.stopLauncher();
            } else {
                voltage = robot.startLauncher();
            }
            launchPress = runtime.seconds();
        }
        telemetry.addData("Battery:" , "Voltage: %.2f", voltage);
        boolean launcherIsReady = false;
        try {
            launcherIsReady = robot.adjustLaunchSpeed(this);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        telemetry.addData("Launcher Power:", "Launcher %s ready, Power set to %.2f",
                launcherIsReady ? "IS" : "IS NOT", robot.launchMotor.getPower());

        if (gamepad2.x && (decreaseSpeed +2) < runtime.seconds()){
            robot.launchMotor.setPower(robot.launchMotor.getPower() - 0.1);
            decreaseSpeed = runtime.seconds();
        }


        if (gamepad2.b && (decreaseSpeed +2) < runtime.seconds()){
            robot.launchMotor.setPower(robot.launchMotor.getPower() + 0.1);
            decreaseSpeed = runtime.seconds();

        }

        if (gamepad2.y && (servoUpTime + 2000) < runtime.milliseconds()) {
            telemetry.addData("Status", "Debug 1 at: " + runtime.toString());
            if (robot.launchServo.getPosition() == robot.launchliftStart) {
                robot.launchServo.setPosition(robot.launchliftDeploy);
            } else {
                robot.launchServo.setPosition(robot.launchliftStart);
            }
            servoUpTime = runtime.milliseconds();
        }
        if (robot.launchServo.getPosition() == robot.launchliftDeploy && servoUpTime + 500 < runtime.milliseconds()) {
            robot.launchServo.setPosition(robot.launchliftStart);
        }
        telemetry.addData("LaunchSpeed", robot.launchMotor.getMaxSpeed());
        telemetry.addData("LaunchPower", robot.launchMotor.getPower());
        int curEncoderValue = robot.launchMotor.getCurrentPosition();
        double curEncoderTime = runtime.milliseconds();
        telemetry.addData("LaunchEncoder", "Current %d / Previous %d / Diff %d",
                Math.abs(curEncoderValue), Math.abs(PrevEncoderValue), Math.abs(curEncoderValue - PrevEncoderValue));
        telemetry.addData("LaunchTiming", "Current %.0f / Previous %.0f / Diff %.0f",
                curEncoderTime, PrevEncoderTime, curEncoderTime-PrevEncoderTime);
        PrevEncoderTime = curEncoderTime;
        PrevEncoderValue = curEncoderValue;

    }
}





