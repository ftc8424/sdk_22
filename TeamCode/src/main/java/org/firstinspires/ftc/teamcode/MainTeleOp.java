
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="MainTeleOp", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class MainTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftWheel, rightWheel;
    DcMotor hitter;
    Servo leftFlap, rightFlap;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

       /* eg: Initialize the hardware variables. Note that the strings used here as parameters
        * to 'get' must correspond to the names assigned during the robot configuration
        * step (using the FTC Robot Controller app on the phone).
        */
        // leftMotor  = hardwareMap.dcMotor.get("left_drive");
        // rightMotor = hardwareMap.dcMotor.get("right_drive");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)

        leftWheel = hardwareMap.dcMotor.get("leftWheel");
        rightWheel = hardwareMap.dcMotor.get("rightWheel");
        hitter = hardwareMap.dcMotor.get("hitter");
        leftFlap = hardwareMap.servo.get("leftFlap");
        rightFlap = hardwareMap.servo.get("rightFlap");


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            if (Math.abs(gamepad1.left_stick_y) > 0.05) {
                leftWheel.setPower(gamepad1.left_stick_y);
            }
            else {
                leftWheel.setPower(0);
            }

            if (Math.abs(gamepad1.right_stick_y) > 0.05) {
                rightWheel.setPower(gamepad1.right_stick_y);
            }
            else {
                rightWheel.setPower(0);
            }


            if (gamepad1.right_bumper) {
                hitter.setPower(1);
            }
            else
            {
                hitter.setPower(0);
            }


            if (gamepad1.left_bumper) {
                if (leftFlap.getPosition() > 0.5 && rightFlap.getPosition() < 0.5)
                {
                    leftFlap.setPosition(0);
                    rightFlap.setPosition(1);
                }
                else
                {
                    leftFlap.setPosition(1);
                    rightFlap.setPosition(0);
                }
            }



            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);
        }
    }
}

