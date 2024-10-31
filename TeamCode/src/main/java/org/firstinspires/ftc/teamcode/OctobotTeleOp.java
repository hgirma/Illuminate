package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Octobot v1", group = "Linear OpMode")
public class OctobotTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private Octobot bot = null;

    @Override
    public void runOpMode() {
        // Initialize Octobot specific hardware variables
        // Octobot specific variables
        Servo claw = hardwareMap.get(Servo.class, "claw"); // servo motor
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm"); // linear slide
        DcMotor wormGear = hardwareMap.get(DcMotor.class, "wormGear");

        // initialize bot and pass in the motors
        bot = new Octobot(claw, arm, wormGear);

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

            telemetry.addData("arm position: ", bot.getArmPosition());
            telemetry.addData("arm locked: ", bot.getArmLocked());
            telemetry.addData("wormGear position: ", bot.getWormGearPosition());
            telemetry.update();
        }
    }

    private void processOctobotCommands() {
        // read the trigger values
        float leftTrigger = gamepad1.left_trigger;
        float rightTrigger = gamepad1.right_trigger;

        // set the power to the value of the trigger to control the speed depending on how far
        // the buttons are pressed. this will allow fine control over the motor
        if (leftTrigger > 0) {
            bot.moveWormGearUp(leftTrigger);
        } else if (rightTrigger > 0) {
            bot.moveWormGearDown(rightTrigger);
        } else {
            bot.stopWormGear();
        }

        // close claw of servo when left bumper is clicked
        if (gamepad1.left_bumper) {
            bot.closeClaw();
        }

        // open claw of servo when left bumper is clicked
        if (gamepad1.right_bumper) {
            bot.openClaw();
        }

        // set direction of arm motor to forward when Y is pressed
        if (gamepad1.y) {
            bot.extendArm();
        }

        // set direction of arm motor to reverse when Y is pressed
        if (gamepad1.a) {
            bot.retractArm();
        }

        // if Y and A button is NOT pressed and the arm is not locked, lock it in position
        if (!gamepad1.y && !gamepad1.a && !bot.getArmLocked()) {
            bot.lockArm();
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
