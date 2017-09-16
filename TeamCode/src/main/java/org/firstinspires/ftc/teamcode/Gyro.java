package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by FTC8424 on 2/19/2017.
 */
@Autonomous(name = "Gyro", group = "RedSide")

public class Gyro extends LinearOpMode {
    HardwareHelper robot = new HardwareHelper(FULLAUTO);



    @Override
    public void runOpMode() throws InterruptedException {

        robot.robot_init(hardwareMap);

        //int Accumulated;
        //double turnspeed = 0.5;


        robot.gyro.calibrate();
        while(!isStopRequested() && robot.gyro.isCalibrating()){
            telemetry.addData("Init:", "Calibrating");
            telemetry.update();
        }
        telemetry.addData("Init:", "Calibrated!!");
        telemetry.update();

        waitForStart();
        while (robot.gyro.isCalibrating()) {

        }

        for (int i = 0; i < 10; i++) {
            int Accumulated = robot.gyro.getIntegratedZValue();
            telemetry.addData("Gyro:", "Z Value: %d", Accumulated);
            telemetry.update();
            sleep(1000);
        }

        while (opModeIsActive()) {
//            robot.encoderDrive(this, 0.35, 24,24,10);
           turnAbsolute(this, 35);
            sleep(1000);
            turnAbsolute(this, 0);
            sleep(1000);
        }

    }

    public void turnAbsolute(LinearOpMode caller, int target) throws InterruptedException {
        int Accumulated = robot.gyro.getIntegratedZValue();
        double turnspeed = 0.1;

        while (opModeIsActive() && Math.abs(Accumulated - target) > 3) {
            if (Accumulated > target) {

                robot.leftMidDrive.setPower(-turnspeed);
                robot.rightMidDrive.setPower(turnspeed);
                robot.leftBackDrive.setPower(-turnspeed);
                robot.rightBackDrive.setPower(turnspeed);
            }

            if (Accumulated < target) {

                robot.leftMidDrive.setPower(turnspeed);
                robot.rightMidDrive.setPower(-turnspeed);
                robot.leftBackDrive.setPower(turnspeed);
                robot.rightBackDrive.setPower(-turnspeed);
            }

            idle();
            Accumulated = robot.gyro.getIntegratedZValue();
            caller.telemetry.addData("Gyro:", "Accumulated Z:  %d", Accumulated);
            caller.telemetry.update();
        }
        robot.leftMidDrive.setPower(0);
        robot.rightMidDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
    }
}