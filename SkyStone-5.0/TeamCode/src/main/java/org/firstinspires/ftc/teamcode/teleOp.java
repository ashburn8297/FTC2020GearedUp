package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static android.os.SystemClock.sleep;


@TeleOp(name = "Mecanum")
public class teleOp extends OpMode {
    robotBase robot = new robotBase();
    ElapsedTime runtime = new ElapsedTime();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    private double boost = 1.0;

    public void init() {
        /*
         * Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry, false);
        sleep(100);



        //Start the robot at zero power, using encoders, and float zero power
        robot.brake();
        robot.runUsingEncoders();
        robot.baseBrake();

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        if(gamepad1.right_trigger > .05) {
            boost = 1 + (gamepad1.right_trigger * .75);
        }
        else{
            boost = 1.0;
        }

        robot.mecanum(gamepad1.left_stick_x * boost, -gamepad1.left_stick_y * boost, gamepad1.right_stick_x);

        //------------------------------------------------------------------------------------------
        telemetry.addLine("Front Motor Powers")
                .addData(" FL", "%.2f", robot.FLD.getPower())
                .addData("FR", "%.2f", robot.FRD.getPower());
        telemetry.addLine("Rear Motor Powers")
                .addData(" RL", "%.2f", robot.RLD.getPower())
                .addData("RR", "%.2f", robot.RRD.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("Stopped after", runtime.seconds());
        telemetry.update();
    }
}
