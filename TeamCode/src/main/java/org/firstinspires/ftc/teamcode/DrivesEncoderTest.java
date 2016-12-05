package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLTELEOP;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.LAUNCHTEST;

/**
 * Created by FTC8424 on 11/16/2016.
 */
@Autonomous(name = "Drives Encoder Test", group = "Tests")
public class DrivesEncoderTest extends LinearOpMode {

    HardwareHelper robot = new HardwareHelper(FULLAUTO);



    @Override
    public void runOpMode() throws InterruptedException {

        robot.robot_init(hardwareMap);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftMidDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Init:" ,"Waiting for start");
        telemetry.update();
        telemetry.addLine("Battery")
                .addData("Volt:", " %.3f", robot.getVoltage());
        telemetry.update();
        while ( !isStarted() ) {
            telemetry.addData("Volt:", " %.3f", robot.getVoltage());
            telemetry.update();
        }

        robot.autoLauncher(this);       // see values

        telemetry.addLine("ENCVALUES");
        while ( opModeIsActive() ) {
            telemetry.addData("ENCVALUES", "LB: %7d", robot.leftBackDrive.getCurrentPosition())
                    .addData("ENCVALUES", "RB: %7d", robot.rightBackDrive.getCurrentPosition())
                    .addData("ENCVALUES", "LM: %7d", robot.leftMidDrive.getCurrentPosition())
                    .addData("ENCVALUES", "RM: %7d", robot.rightMidDrive.getCurrentPosition());
            telemetry.update();
        }
   }
}