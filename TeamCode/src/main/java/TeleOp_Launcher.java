import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareHelper;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.LAUNCHTEST;

/**
 * Created by avc on 10/25/2016.
 */

public class TeleOp_Launcher extends OpMode {



    private ElapsedTime runtime = new ElapsedTime();
    private HardwareHelper robot = new HardwareHelper(LAUNCHTEST);
    @Override
    public void init() {
    robot.robot_init(hardwareMap);
    }

    @Override
    public void loop() {
     if (gamepad1.a = true && launchMotor > 0) {
            SetPower(0);
            launchMotor = 0;
        } else if (gamepad1.a && launchMotor == 0)

    }
}
