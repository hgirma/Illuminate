package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Octobot {
    private enum ClawPosition {
        CLOSED, // claw is open - 0
        OPEN // claw is open - 1
    }

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
    public void moveWormGear(DcMotor.Direction direction, float speed) {
        wormGear.setDirection(direction);
        wormGear.setPower(speed);

        octoboState.setWormGearPosition(wormGear.getCurrentPosition());
    }

    public void stopWormGear() {
        wormGear.setPower(0);
    }

    public void openClaw() {
        claw.setPosition(ClawPosition.OPEN.ordinal());
    }

    public void closeClaw() {
        claw.setPosition(ClawPosition.CLOSED.ordinal());
    }

    public void moveArm(DcMotor.Direction direction) {
        arm.setDirection(direction);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(ARM_SPEED);

        octoboState.setArmPosition(arm.getCurrentPosition());
        octoboState.setArmLocked(false);
    }

    public void lockArm() {
        // if arm is already locked, return
        if (octoboState.getArmLocked()) {
            return;
        }

        int currentPosition = arm.getCurrentPosition(); // Get current position

        arm.setTargetPosition(currentPosition); // Set target to current position
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
