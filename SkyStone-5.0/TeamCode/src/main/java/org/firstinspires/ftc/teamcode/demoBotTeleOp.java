package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp(name="Demo Bot") //tam
public class demoBotTeleOp extends OpMode {
    public DcMotor LD                  = null; //Front Left Drive Motor, "FLD"
    public DcMotor RD                  = null; //Front Right Drive Motor, "FRD"
    public void init() {
        LD = hardwareMap.get(DcMotor.class, "FLD");
        RD = hardwareMap.get(DcMotor.class, "FRD");

        LD.setDirection(DcMotor.Direction.REVERSE);
        RD.setDirection(DcMotor.Direction.FORWARD);

        LD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LD.setPower(0);
        RD.setPower(0);
    }
    @Override
    public void loop(){
        LD.setPower(-gamepad1.left_stick_y);
        RD.setPower(-gamepad1.right_stick_y);
    }
}
