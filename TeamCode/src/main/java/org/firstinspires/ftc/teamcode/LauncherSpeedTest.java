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
        robot.launchMotor.setMaxSpeed(3000);
    }

    @Override
    public void loop() {

        if (gamepad2.a && (launchPress + 2) < runtime.seconds()) {
            if ( robot.launchMotor.getPower() > 0.0 ) {
                robot.launchMotor.setPower(0);
            } else {
                robot.launchMotor.setPower(0.65);
            }
            launchPress = runtime.seconds();
        }

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
        telemetry.addData("LaunchEncoder", "%.2f / %.2f", (float)Math.abs(PrevEncoderValue - curEncoderValue), (float)Math.abs(curEncoderTime - PrevEncoderTime));
        PrevEncoderTime = curEncoderTime;
        PrevEncoderValue = curEncoderValue;

    }
}





