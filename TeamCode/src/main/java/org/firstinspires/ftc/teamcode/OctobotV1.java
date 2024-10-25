package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Octobot v1", group = "Linear OpMode")
public class OctobotV1 extends LinearOpMode {

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

    private OctoboState state = new OctoboState();

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

            ProcessOctobotCommands();

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

    private void ProcessOctobotCommands() {
        // set motor direction based on gamepad input
        if (gamepad1.dpad_down) {
            wormGear.setDirection(DcMotor.Direction.FORWARD);
        }

        if (gamepad1.dpad_up) {
            wormGear.setDirection(DcMotor.Direction.REVERSE);
        }

        if (gamepad1.y) {
            arm.setDirection(DcMotor.Direction.FORWARD);
        }

        if (gamepad1.a) {
            arm.setDirection(DcMotor.Direction.REVERSE);
        }

        if (gamepad1.left_bumper) {
            claw.setPosition(0);
        }

        if (gamepad1.right_bumper) {
            claw.setPosition(1);
        }

        // give power to motors based on gamepad input
        arm.setPower(gamepad1.y ? 1 : 0);
        arm.setPower(gamepad1.a ? 1 : 0);
        wormGear.setPower(gamepad1.dpad_up ? 1 : 0);
        wormGear.setPower(gamepad1.dpad_down ? 1 : 0);
    }
}
