package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Octobot {

    // Octobot specific variables
    private final Servo claw;
    private final DcMotor arm;
    private final DcMotor wormGear;

    // speed settings for motors
    final double ARM_SPEED = 1;

    // to capture various states of the motors
    OctoboState octoboState = new OctoboState();

    public Octobot(Servo claw, DcMotor arm, DcMotor wormGear) {
        // Initialize Octobot specific hardware variables
        this.claw = claw;
        this.arm = arm;
        this.wormGear = wormGear;
    }

    /**
     * @param speed the amount of power to give the motor
     */
    public void moveWormGearDown(float speed) {
        wormGear.setDirection(DcMotor.Direction.FORWARD);
        wormGear.setPower(speed);
        octoboState.setWormGearPosition(wormGear.getCurrentPosition());
    }

    /**
     * @param speed the amount of power to give the motor
     */
    public void moveWormGearUp(float speed) {
        wormGear.setDirection(DcMotor.Direction.REVERSE);
        wormGear.setPower(speed);
        octoboState.setWormGearPosition(wormGear.getCurrentPosition());
    }

    public void stopWormGear() {
        wormGear.setPower(0);
    }

    public void openClaw() {
        claw.setPosition(1);
    }

    public void closeClaw() {
        claw.setPosition(0);
    }

    public void extendArm() {

        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //unlock to it moves freely
        arm.setPower(ARM_SPEED);

        octoboState.setArmPosition(arm.getCurrentPosition());
        octoboState.setArmLocked(false);
    }

    public void retractArm() {
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //unlock to it moves freely
        arm.setPower(ARM_SPEED);

        octoboState.setArmPosition(arm.getCurrentPosition());
        octoboState.setArmLocked(false);
    }

    public void lockArm() {
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // lock in place
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        octoboState.setArmLocked(true);
    }

    public double getArmPosition() {
        return octoboState.getArmPosition();
    }

    public boolean getArmLocked() {
        return octoboState.getArmLocked();
    }

    public double getWormGearPosition() {
        return octoboState.getWormGearPosition();
    }
}
