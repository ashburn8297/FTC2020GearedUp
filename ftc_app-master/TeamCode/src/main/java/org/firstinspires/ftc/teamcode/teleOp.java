package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum Test")
public class teleOp extends OpMode {
    robotBase robot       = new robotBase();
    ElapsedTime runtime   = new ElapsedTime();

    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //Start the robot at zero power, using encoders, and float zero power
        robot.brake();
        robot.runUsingEncoders();
        robot.baseFloat();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop(){

        robot.mecanumController(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad2.right_stick_x);

        telemetry.addData("Time", runtime.seconds());
        telemetry.addData("Controller X", gamepad1.left_stick_x);
        telemetry.addData("Controller Y", gamepad1.left_stick_y);
        telemetry.addData("FL", robot.FLD.getPower());
        telemetry.addData("FR", robot.FRD.getPower());
        telemetry.addData("RL", robot.RLD.getPower());
        telemetry.addData("RR", robot.RRD.getPower());
        telemetry.update();
    }

    @Override
    public void stop(){
    }
}
