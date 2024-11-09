package org.firstinspires.ftc.teamcode.octobotOld;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This is meant to be a library!
 * Do not copy and edit the code
 * Import it in a new java class!
 *
 * DO NOT MODIFY
 */

public class OctobotAutoEssentials extends LinearOpMode {
    public DcMotor LF; // LeftFront Motor
    public DcMotor LB; // LeftBack Motor
    public DcMotor RF; // RightFront Motor
    public DcMotor RB; // RightBack Motor

    public DcMotor RS; // RightSlide Motor

    public Servo FBS; //FourBarLinkage Servo
    public Servo CS; //Claw Servo

    public BNO055IMU imu;

    public int tickRotation;
    public int tickAcceptance;

    public OpenCvWebcam webcam;

    public OctobotAutoEssentials(int tickRotation, int tickAcceptance) {
        this.tickRotation = tickRotation;
        this.tickAcceptance = tickAcceptance;
    }

    public void initialize() {
        LF = hardwareMap.get(DcMotor.class, "frontLeft");
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) LF).setTargetPositionTolerance(tickAcceptance);

        LB = hardwareMap.get(DcMotor.class, "backLeft");
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) LB).setTargetPositionTolerance(tickAcceptance);

        RF = hardwareMap.get(DcMotor.class, "frontRight");
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) RF).setTargetPositionTolerance(tickAcceptance);

        RB = hardwareMap.get(DcMotor.class, "backRight");
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) RB).setTargetPositionTolerance(tickAcceptance);


//        RS = hardwareMap.get(DcMotor.class, "RightSlide");
//        RS.setDirection(DcMotorSimple.Direction.REVERSE);
//        RS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        FBS = hardwareMap.get(Servo.class, "FourBarServo");
//        CS = hardwareMap.get(Servo.class, "ClawServo");
//
        imu = hardwareMap.get(BNO055IMU.class, "imu");


    }


    public void reset() {
        stopDrivetrain();

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void stopDrivetrain() {
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
    }

    public void initIMU() {
        BNO055IMU.Parameters imuParameters;

        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Express acceleration as m/s^2.
        imuParameters.loggingEnabled = false;
        // Disable logging.
        imu.initialize(imuParameters);
        // Initialize imu.
    }


    public void goRotations(double rotations, double power) {
        reset();
        int rotationsInTicks = (int) (rotations * tickRotation);

        LF.setTargetPosition(rotationsInTicks);
        LB.setTargetPosition(rotationsInTicks);
        RF.setTargetPosition(rotationsInTicks);
        RB.setTargetPosition(rotationsInTicks);

        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LF.setPower(power);
        LB.setPower(power);
        RF.setPower(power);
        RB.setPower(power);

        while (LF.isBusy() ||
                LB.isBusy() ||
                RF.isBusy() ||
                RB.isBusy()
        ) {
            telemetry.addData("LF Current Position", LF.getCurrentPosition());
            telemetry.addData("LB Current Position", LB.getCurrentPosition());
            telemetry.addData("RF Current Position", RF.getCurrentPosition());
            telemetry.addData("RB Current Position", RB.getCurrentPosition());
            telemetry.update();
        }

        stopDrivetrain();
    }

    public void strafeLeft(double distance, double power) {
        goDiagonal(-distance, distance, distance, -distance, power, power, power, power);
    }

    public void strafeRight(double distance, double power) {
        goDiagonal(distance, -distance, -distance, distance, power, power, power, power);
    }

    public void goDiagonal(double dLF, double dLB, double dRF, double dRB, double sLF, double sLB, double sRF, double sRB) {
        reset();
        LF.setTargetPosition((int) (560 * dLF));
        LB.setTargetPosition((int) (560 * dLB));
        RB.setTargetPosition((int) (560 * dRB));
        RF.setTargetPosition((int) (560 * dRF));
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setPower(sLF);
        LB.setPower(sLB);
        RB.setPower(sRB);
        RF.setPower(sRF);
        while (!(!LF.isBusy() && !LB.isBusy() && !RB.isBusy() && !RF.isBusy())) {
            // Put loop blocks here.
        }
        stopDrivetrain();
    }

    public void turnAngle(int inputangle, double speed) {
        initialize();
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < inputangle) {
            while (!(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle >= inputangle)) {
                LF.setPower(-speed);
                LB.setPower(-speed);
                RB.setPower(speed);
                RF.setPower(speed);
            }
        } else if (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > inputangle) {
            while (!(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle <= inputangle)) {
                LF.setPower(speed);
                LB.setPower(speed);
                RB.setPower(-speed);
                RF.setPower(-speed);
            }
        }
        LF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
    }


    @Override
    public void runOpMode() {
    }
}

