package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by FTC8424 on 1/19/2017.
 */

@TeleOp(name = "Gyro_Sensor", group = "Sensor")
public class Gyro_Sensor extends LinearOpMode{
    //Gyro Calibrating
    HardwareHelper robot = new HardwareHelper(FULLAUTO);
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.robot_init(hardwareMap);
        // Hardware Device Object
        double  driveSpeed = .75;
        ModernRoboticsI2cGyro gyro;
        int xVal, yVal = 0;     // Gyro rate Values
        int largeHeading = 0;              // Gyro integrated heading
        int currentHeading = 1;
       //boolean lastResetState = false;
        //boolean curResetState  = false;


//        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        robot.gyro.calibrate();

        // make sure the gyro is calibrated.
        while (! isStopRequested() && robot.gyro.isCalibrating())  {
            telemetry.addData(">", "Gyro Calibrating. Do Not move!");
            telemetry.update();

    }
        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        // wait for the start button to be pressed.
        waitForStart();

        //robot.encoderDrive(this, driveSpeed, 6.5, -6.5, 10);

        xVal = robot.gyro.rawX();
        yVal = robot.gyro.rawY();


        //telemetry.addData(">", "Press A & B to reset Heading.");
        telemetry.addData("2", "X val %03d", xVal);
        telemetry.addData("3", "Y val %03d", yVal);
        telemetry.update();
        sleep(2000);
        idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop

//        xVal = robot.gyro.rawX();
//        yVal = robot.gyro.rawY();
//
//
//        //telemetry.addData(">", "Press A & B to reset Heading.");
//        telemetry.addData("2", "X val %03d", xVal);
//        telemetry.addData("3", "Y val %03d", yVal);
//        telemetry.update();
//        sleep(2000);
//        idle();

        }
}