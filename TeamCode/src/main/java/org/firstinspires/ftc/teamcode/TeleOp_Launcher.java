package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareHelper;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.LAUNCHTEST;

/**
 * Created by avc on 10/25/2016.
 */

@TeleOp(name="Launcher", group="Tests")

public class TeleOp_Launcher extends OpMode {

    double launchPress;
    double lastStateChange;
    int launcherState;
    private ElapsedTime runtime = new ElapsedTime();
    private HardwareHelper robot = new HardwareHelper(LAUNCHTEST);

    @Override
    public void init() {
        robot.robot_init(hardwareMap);
    }

    @Override
    public void loop() {


        if (gamepad2.a && launcherState > 0 && launchPress + 2 < runtime.seconds()) {
            robot.launchMotor1.setPower(0);
            launcherState = 0;
            launchPress = runtime.seconds();
        }
        if (gamepad2.a && launcherState > 0 && launchPress + 2 < runtime.seconds()) {
            robot.launchMotor2.setPower(0);
            launcherState = 0;
            launchPress = runtime.seconds();
        }
        if (gamepad2.a && launcherState == 0 && launchPress + 2 < runtime.seconds()) {
            robot.launchMotor1.setPower(.65);
            launchPress = runtime.seconds();
            launcherState = 1;
            lastStateChange = runtime.milliseconds();
        }
        if (gamepad2.a && launcherState == 0 && launchPress + 2 < runtime.seconds()) {
            robot.launchMotor2.setPower(.65);
            launchPress = runtime.seconds();
            launcherState = 1;
            lastStateChange = runtime.milliseconds();
        }
        if (launcherState > 0 && lastStateChange + 500 < runtime.milliseconds()) {

            if (launcherState < 5) {
                robot.launchMotor1.setPower(robot.launchMotor1.getPower() + .1);
                lastStateChange = runtime.milliseconds();
                launcherState++;
            }

            if (launcherState < 5) {
                robot.launchMotor2.setPower(robot.launchMotor2.getPower() + .1);
                lastStateChange = runtime.milliseconds();
                launcherState++;
            }



        }
        if(Math.abs(gamepad2.right_stick_y) > .01) {
            robot.manipMotor.setPower(gamepad2.right_stick_y);
        }
        else {
            robot.manipMotor.setPower(0);
        }
}



                }