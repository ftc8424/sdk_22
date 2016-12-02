package org.firstinspires.ftc.teamcode;
//
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by Devan on 10/9/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto_Red_LowPower", group = "Sensor")
public class Auto_Red_LowPower extends LinearOpMode{
//Trollbot is 14.5 inches

    HardwareHelper robot = new HardwareHelper(FULLAUTO);
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

//        waitForStart();
//
//     hsvValues is an array that will hold the hue, saturation, and value information.
//        float hsvValues[] = {0F, 0F, 0F};
//
//    // values is a reference to the hsvValues array.
//        final float values[] = hsvValues;

//        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
//
//    // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        double  driveSpeed = .5;


        robot.robot_init(hardwareMap);

        robot.color.enableLed(true);
        idle();
        sleep(1000L);
        robot.color.enableLed(false);
        idle();

        telemetry.addData("Init:" ,"Waiting for start");
        telemetry.update();
        idle();
        waitForStart();


//        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
//
//      // send the info back to driver station using telemetry function.
        // Move forward to shoot
        //robot.encoderDrive(this, driveSpeed,-3, -3,10);
        // shooting
        robot.autoLauncher(this, 0.95);
        // Backing up
        robot.encoderDrive(this, driveSpeed, -13,-13,10);
        // Figure out right turn for 135 deg.
        robot.encoderDrive(this,driveSpeed, 18, -18, 10);
        robot.encoderDrive(this, driveSpeed, -6, 6, 5);


//    robot.encoderDrive(this, driveSpeed, -9.75, 9.75, 10);
        //Moving to beacon
        robot.encoderDrive(this,driveSpeed, 31, 31, 10);


        //Aligning the robot at the 1st beacon
       robot.encoderDrive(this, driveSpeed, -4, 4, 5);

        //Driving towards the first beacon
       robot.encoderDrive(this, driveSpeed, 9.25, 9.25, 5);
        //1




        telemetry.addData("Color: ", "Red %d Blue %d Green %d", robot.color.red(), robot.color.blue(), robot.color.green());
//        telemetry.update();
//
//        telemetry.addData("Clear", robot.color.alpha());
//        telemetry.update();
//        telemetry.addData("Red  ", robot.color.red());
//        telemetry.update();
//        telemetry.addData("Green", robot.color.green());
//        telemetry.update();
//        telemetry.addData("Blue ", robot.color.blue());
//        telemetry.update();

        //Logic for pressing the red button, when we are the red alliance (at the 1st beacon)
        if (robot.color.red() > 0 && robot.color.red() > robot.color.blue()) {
            robot.leftPush.setPosition(robot.lpushDeploy);
        } else if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()){
            robot.rightPush.setPosition(robot.rpushDeploy);
        } else if (robot.color.red() == robot.color.blue()){
            telemetry.addData("ColorDecision: ", "Not Pressing");
            telemetry.update();
        }
        idle();
        sleep(1000);
        robot.leftPush.setPosition(robot.lpushStart);
        robot.rightPush.setPosition(robot.rpushStart);
        idle();

        //Backing away from Beacon 1
        robot.encoderDrive(this, driveSpeed, -7, -7, 5);
//        //4.66
        //Turning right towards beacon 2
        robot.encoderDrive(this, driveSpeed, 11, -11, 10);
//        //6.66
//
//
        // Drive to beacon 2
        robot.encoderDrive(this, driveSpeed, 30, 30, 10);
//        //34.66
//
//
        //Turn left @ Beacon 2
        robot.encoderDrive(this, driveSpeed, -11, 11, 10);
//        //8
//
        // Forward to Beacon 2
        robot.encoderDrive(this, driveSpeed, 7, 7, 10);
//        //10


        // Logic for pressing red, when we are red
        if (robot.color.red() > 0 && robot.color.red() > robot.color.blue()) {
            robot.leftPush.setPosition(robot.lpushDeploy);
        } else if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()){
            robot.rightPush.setPosition(robot.rpushDeploy);
        } else if (robot.color.red() == robot.color.blue()){
            telemetry.addData("ColorDecision: ", "Not Pressing");
            telemetry.update();
        }
        idle();
        sleep(1000);
        robot.leftPush.setPosition(robot.lpushStart);
        robot.rightPush.setPosition(robot.rpushStart);
        idle();

        robot.encoderDrive(this, driveSpeed, -11, -11, 5);
            //Backing up from Beacon 2

        robot.encoderDrive(this, driveSpeed, 5, -5,10);
        //3.33
            //Turning towards Cap


        robot.encoderDrive(this, driveSpeed, -60, -60, 10);
        //40            //Moving towards Cap ball

        robot.encoderDrive(this, driveSpeed, -24, 24, 10);

        robot.encoderDrive(this, driveSpeed, 65, 65, 10);



    }
}