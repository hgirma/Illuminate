package org.firstinspires.ftc.teamcode.octobotOld;

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

    final double WARM_GEAR_SPEED = 1;

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

        wormGear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wormGear.setDirection(direction);

        wormGear.setPower(speed);
    }

    public void moveWorkGearToPosition(int position) {

        wormGear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wormGear.setTargetPosition(position);

        octoboState.setWormGearPosition(position);

        wormGear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        double maxPower = 1;
//        double minPower = .8;
//
//        while (wormGear.isBusy()) {
//            int currentPosition = wormGear.getCurrentPosition();
//            int distanceToTarget = Math.abs(position - currentPosition);
//
//            // Calculate power based on distanceToTarget
//            double power = maxPower * ((double) distanceToTarget / position);
//
//            // Ensure power does not fall below minPower
//            power = Math.max(power, minPower);
//
//            // Set the motor power
//            wormGear.setPower(power);
//        }

        wormGear.setPower(WARM_GEAR_SPEED);
    }

    public void stopWormGear() {
        wormGear.setPower(0);
    }

    public void moveArm(DcMotor.Direction direction) {

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setDirection(direction);

        arm.setPower(ARM_SPEED);

        octoboState.setArmLocked(false);
    }

    public void moveArmToPosition(int position) {

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setTargetPosition(position);

        octoboState.setArmPosition(position);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPower(ARM_SPEED);
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

    public void openClaw() {
        claw.setPosition(.45);
    }

    public void closeClaw() {
        claw.setPosition(0);
    }

    public void stopClaw() {
        claw.setPosition(.5);
    }

    public void restPositions() {
        octoboState.setArmPosition(0);
        octoboState.setWormGearPosition(0);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
