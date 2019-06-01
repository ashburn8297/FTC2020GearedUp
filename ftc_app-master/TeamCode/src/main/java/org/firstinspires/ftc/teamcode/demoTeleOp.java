package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//geet test
@TeleOp(name="Demo Bot") //tam
public class demoTeleOp extends OpMode {
    public DcMotor FLD                  = null; //Front Left Drive Motor, "FLD"
    public DcMotor FRD                  = null; //Front Right Drive Motor, "FRD"
    public void init() {
        FLD = hardwareMap.get(DcMotor.class, "FLD");
        FRD = hardwareMap.get(DcMotor.class, "FRD");

        FLD.setDirection(DcMotor.Direction.REVERSE);
        FRD.setDirection(DcMotor.Direction.FORWARD);

        FLD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FLD.setPower(0);
        FRD.setPower(0);
    }
    @Override
    public void loop(){
        FLD.setPower(-gamepad1.left_stick_y);
        FRD.setPower(-gamepad1.right_stick_y);
    }
}
