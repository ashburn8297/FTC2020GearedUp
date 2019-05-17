package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Strafe Test")
public class auto extends LinearOpMode {
    robotBase robot       = new robotBase();
    ElapsedTime runtime   = new ElapsedTime();

    @Override
    public void runOpMode(){
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry);

        //Start the robot at zero power, using encoders, and float zero power
        robot.brake();
        robot.runUsingEncoders();
        robot.baseFloat();

        //calibrate gyro
        runtime.reset();
        while (!isStopRequested() && robot.navxMicro.isCalibrating())  {
            telemetry.addData("Calibrating NavX", "%s", Math.round(runtime.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating())  {
            telemetry.addData("Calibrating MR", "%s", Math.round(runtime.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();
        //------------------------------------------------------------------------------------------
        runtime.reset();

        robot.translate(0, 40, 10.0, 1.0, true,auto.this, telemetry);
        sleep(500);

        robot.turn(45, .25, 4.0, true,auto.this, telemetry);
        sleep(500);

        stop();

    }
}
