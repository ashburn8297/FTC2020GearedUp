package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class robotBase {

    /*Public hardware members*/
    public DcMotor FLD          = null; //Front Left Drive Motor, "FLD"
    public DcMotor FRD          = null; //Front Right Drive Motor, "FRD"
    public DcMotor RLD          = null; //Rear Left Drive Motor, "RLD"
    public DcMotor RRD          = null; //Rear Right Drive Motor, "RRD"

    /*Local opMode members*/
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    //Empty constructor for object creation.
    public robotBase(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        //Initialize the four drive base motors
        FLD = hwMap.get(DcMotor.class, "FLD");
        FRD = hwMap.get(DcMotor.class, "FRD");
        RLD = hwMap.get(DcMotor.class, "RLD");
        RRD = hwMap.get(DcMotor.class, "RRD");

        //Set the drive base motors so that +1 drives forward

        /*It's possible these values are reversed*/
        FLD.setDirection(DcMotor.Direction.FORWARD);
        FRD.setDirection(DcMotor.Direction.REVERSE);
        RLD.setDirection(DcMotor.Direction.FORWARD);
        RRD.setDirection(DcMotor.Direction.REVERSE);

        //Stop all motors when robot is initialized
        FLD.setPower(0);
        FRD.setPower(0);
        RLD.setPower(0);
        RRD.setPower(0);

        //Set all motors so that they begin without encoders enabled
        FLD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RLD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*Control the mecanum drive base with a controller's x input (double X1)
    and a controller's y input (double Y1) to translate,
    and a second x input (double X2) to rotate*/
    public void mecanumController(double X1, double Y1, double X2){
        //Find the magnitude of the controller's input
        double r = Math.hypot(X1, X2);

        double robotAngle = Math.atan2(Y1, X1) - Math.PI / 4;
        double rightX = X2;

        //double vX represents the velocities sent to each motor
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        FLD.setPower(v1);
        FRD.setPower(v2);
        RLD.setPower(v3);
        RRD.setPower(v4);
    }

    //https://github.com/trc492/FtcSamples/blob/master/Ftc3543Lib/src/main/java/trclib/TrcPidDrive.java
}
