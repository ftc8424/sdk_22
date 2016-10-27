import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareHelper;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.LAUNCHTEST;

/**
 * Created by avc on 10/25/2016.
 */

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
        /*
         * Mohana:  There's three things wrong with the first IF statement:
         *     1.  You're doing a "gamepad1.a = true" so you're trying to set the value of gamepad1.a which is illegal
         *        This should either be a double equals for comparison, or just drop the "= true" since
         *        the value of gamepad1.a is a boolean
         *    2.  You haven't declared the launchMotor variable yet, that's why it's in red
         *        It should probably be the launcherState, since you want to look at the state of the launcher
         *    3.  The setPower() has no reference object, you need to call setPower() on SOMETHING
         *        and you haven't given it anything.  This should be robot.launchMotor.setPower(0);
         *
         *    Also, in general, you'll want to change the IF to be something like this:
         *      if ( gamepad1.a && launcherState > 0 && launchPress+2 < runtime.seconds() )
         *    That way, you're using gamepad-1's A button as a toggle so if the launcher is in an
         *    ON state (launcherState > 0) and the last time the gamepad1.a was pressed is at least
         *    2 seconds ago, then turn off the motor
         *
         *    Read my email for the rest of the description of the variables and such.
         *
         *    -- Coach Galligher
         */
        if (gamepad2.a && launcherState > 0 && launchPress + 2 < runtime.seconds()) {
            robot.launchMotor.setPower(0);
            launcherState = 0;
            launchPress = runtime.seconds();
        }
        if (gamepad2.a && launcherState == 0 && launchPress + 2 < runtime.seconds()) {
            robot.launchMotor.setPower(.1);
            launchPress = runtime.seconds();
            launcherState = 1;
            lastStateChange = runtime.milliseconds();

        }
        if (launcherState > 0 && lastStateChange + 500 < runtime.milliseconds()) {

            if (launcherState < 5) {
                robot.launchMotor.setPower(robot.launchMotor.getPower() + .1);
                lastStateChange = runtime.milliseconds();
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