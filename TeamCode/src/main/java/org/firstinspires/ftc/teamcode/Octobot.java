package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Octobot v1", group = "Linear OpMode")
public class Octobot extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Octobot specific variables
    private Servo claw = null;
    private DcMotor arm = null;
    private DcMotor wormGear = null;

    // speed settings for motors
    final double ARM_SPEED = 1;

    // to capture various states of the motors
    OctoboState octoboState = new OctoboState();

    @Override
    public void runOpMode() {
        // Initialize Octobot specific hardware variables
        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wormGear = hardwareMap.get(DcMotor.class, "wormGear");

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // log the values read from gamepad sticks and triggers
            telemetry.addData("left_stick_x: ", gamepad1.left_stick_x);
            telemetry.addData("left_stick_y: ", gamepad1.left_stick_y);

            telemetry.addData("right_stick_x: ", gamepad1.right_stick_x);
            telemetry.addData("right_stick_y: ", gamepad1.right_stick_y);

            telemetry.addData("left_trigger: ", gamepad1.left_trigger);
            telemetry.addData("right_trigger: ", gamepad1.right_trigger);

            processOctobotCommands();

            processFtcDrivetrainCommands();

            telemetry.addData("arm position: ", octoboState.getArmPosition());
            telemetry.addData("arm locked: ", octoboState.getArmLocked());
            telemetry.addData("wormGear position: ", octoboState.getWormGearPosition());
            telemetry.update();
        }
    }

    private void processOctobotCommands() {
        // read the trigger values
        double leftTrigger = gamepad1.left_trigger;
        double rightTrigger = gamepad1.right_trigger;

        // set forward direction of wormGear motor when value is greater than 0
        if (leftTrigger > 0) {
            wormGear.setDirection(DcMotor.Direction.FORWARD);
        }

        if (rightTrigger > 0) {
            wormGear.setDirection(DcMotor.Direction.REVERSE);
        }

        // set the power to the value of the trigger to control the speed depending on how far
        // the buttons are pressed. this will allow fine control over the motor
        if (leftTrigger > 0) {
            wormGear.setPower(leftTrigger);
            octoboState.setWormGearPosition(wormGear.getCurrentPosition());
        } else if (rightTrigger > 0) {
            wormGear.setPower(rightTrigger);
            octoboState.setWormGearPosition(wormGear.getCurrentPosition());
        } else {
            wormGear.setPower(0);
        }

        // close claw of servo when left bumper is clicked
        if (gamepad1.left_bumper) {
            claw.setPosition(0);
        }

        // open claw of servo when left bumper is clicked
        if (gamepad1.right_bumper) {
            claw.setPosition(1);
        }

        // set direction of arm motor to forward when Y is pressed
        if (gamepad1.y) {
            arm.setDirection(DcMotor.Direction.FORWARD);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //unlock to it moves freely
        }

        // set direction of arm motor to reverse when Y is pressed
        if (gamepad1.a) {
            arm.setDirection(DcMotor.Direction.REVERSE);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //unlock to it moves freely
        }

        // if A or X button is pressed, give power to arm motor to move it
        if (gamepad1.y || gamepad1.a) {
            arm.setPower(ARM_SPEED);
            octoboState.setArmPosition(arm.getCurrentPosition());
        } else {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // lock in place
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            octoboState.setArmLocked(true);
        }
    }

    // ftc provided code to move the drive train
    private void processFtcDrivetrainCommands() {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}
