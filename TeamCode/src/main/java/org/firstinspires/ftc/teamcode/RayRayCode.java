package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="RaynTeleOp", group="Linear OpMode")
public class RayRayCode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor linearLeft = null;
    private DcMotor linearRight = null;

    private DcMotor pitchMotor = null;
    private DcMotor actuator = null;

    private Servo clawPitch = null; // 0
    private static final double CLAW_PITCH_LOWER = 0.4;
    private static final double CLAW_PITCH_UPPER = 1;
    private double clawPitchPosition = CLAW_PITCH_LOWER;

    private Servo claw = null; // 1
    private double clawPosition = 0;

    private boolean isReverse = false;
    private boolean isHanging = false;

    // Variables for toggle functionality
    private boolean toggleClawState = false;
    private boolean previousGamepad2XState = false;
    private static final double CLAW_OPEN_POS = 0.1;
    private static final double CLAW_CLOSE_POS = 0.3;
    private static final int PITCH_TARGET_1 = -4250; // Adjust target positions as needed
    private static final int PITCH_TARGET_2 = -1750;
    private static final double PITCH_AUTO_SPEED = 0.8;
    private boolean pitchMotorAutoMoving = false;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearLeft = hardwareMap.get(DcMotorEx.class, "linearLeft");
        linearRight = hardwareMap.get(DcMotorEx.class, "linearRight");
        linearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLeft.setDirection(DcMotor.Direction.FORWARD);
        linearRight.setDirection(DcMotor.Direction.REVERSE);
        linearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pitchMotor = hardwareMap.get(DcMotorEx.class, "pitchMothy");
        actuator = hardwareMap.get(DcMotorEx.class, "actuator");
        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setDirection(DcMotor.Direction.FORWARD);
        actuator.setDirection(DcMotor.Direction.FORWARD);
        pitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawPitch = hardwareMap.get(Servo.class, "clawPitch");
        claw = hardwareMap.get(Servo.class, "claw");
        openClaw();
        clawPitchGoTo(CLAW_PITCH_LOWER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // --------------------------- WHEELS --------------------------- //

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = Math.pow(-gamepad1.left_stick_y, 3);  // Note: pushing stick forward gives negative value
            double lateral = Math.pow(gamepad1.left_stick_x, 3);
            double yaw = Math.pow(gamepad1.right_stick_x, 3);
            double leftFrontPower = gamepad1.dpad_left ? 1 : axial + lateral + yaw;
            double rightFrontPower = gamepad1.dpad_right ? 1 : axial - lateral - yaw;
            double leftBackPower = gamepad1.dpad_down ? 1 : axial - lateral + yaw;
            double rightBackPower = gamepad1.dpad_up ? 1 : axial + lateral - yaw;

            if (gamepad1.y && !GP1aButtonPreviousState) {
                isReverse = !isReverse;
            }
            GP1aButtonPreviousState = gamepad1.y;
            if (isReverse) { // Reverse everything but turning
                leftFrontPower = -axial - lateral + yaw;
                rightFrontPower = -axial + lateral - yaw;
                leftBackPower = -axial + lateral + yaw;
                rightBackPower = -axial - lateral - yaw;
            }

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Reduced Power Mode
            if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {
                leftFrontPower /= 2;
                rightFrontPower /= 2;
                leftBackPower /= 2;
                rightBackPower /= 2;
            }

            boolean linearOverHalf = linearLeft.getCurrentPosition() > 1500 && linearRight.getCurrentPosition() > 1500;
            DcMotor.ZeroPowerBehavior zeroPowerBehavior = linearOverHalf ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE;
            setDriveMotorsZeroPowerBehavior(zeroPowerBehavior);

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // --------------------------- LINEAR SLIDERS --------------------------- //

            linearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            linearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double linearPower = Math.pow(gamepad2.right_trigger - gamepad2.left_trigger, 3); // Pow3 for larger intervals of slow and fast speeds
            double appliedPower = getAppliedPower(linearPower);
            linearLeft.setPower(appliedPower);
            linearRight.setPower(appliedPower);

            // --------------------------- ACTUATOR & PITCH --------------------------- //

            if (gamepad2.dpad_up) { // Out
                actuator.setPower(1);
            } else if (gamepad2.dpad_down) { // In
                actuator.setPower(-1);
            } else {
                actuator.setPower(0);
            }

            // Handle pitch motor movement
            if (pitchMotorAutoMoving) {
                if (!pitchMotor.isBusy()) {
                    pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    pitchMotorAutoMoving = false;
                }
            } else {
                double pitchPower = Math.pow(-gamepad2.right_stick_y, 3);
                pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pitchMotor.setPower(pitchPower);
            }

            // --------------------------- CLAW PITCH & CLAW --------------------------- //

            if (-gamepad2.left_stick_y > 0 && clawPitchPosition < CLAW_PITCH_UPPER) { // Forwards
                clawPitchPosition += 0.01 * -gamepad2.left_stick_y;
            } else if (-gamepad2.left_stick_y < 0 && clawPitchPosition > CLAW_PITCH_LOWER) { // Backwards
                clawPitchPosition += 0.01 * -gamepad2.left_stick_y;
            }
            clawPitch.setPosition(clawPitchPosition);

            if (gamepad2.left_bumper) { // Open
                openClaw();
            } else if (gamepad2.right_bumper) { // Close
                closeClaw();
            }

            // Handle gamepad2.x toggle for claw and pitch
            if (gamepad2.x && !previousGamepad2XState) {
                toggleClawState = !toggleClawState;

                Thread pitchGoingToTarget;
                if (toggleClawState) {
                    // Open claw, set claw pitch to lower limit, move pitch motor to TARGET_1
                    openClaw();
                    clawPitchGoTo(CLAW_PITCH_LOWER);
                    pitchGoingToTarget = new Thread(() -> pitchGoTo(PITCH_AUTO_SPEED,PITCH_TARGET_1,5));
                } else {
                    // Close claw, set claw pitch to upper limit, move pitch motor to TARGET_2
                    closeClaw();
                    sleep(500); // Wait for claw to close on specimen completely
                    clawPitchGoUpper();
                    pitchGoingToTarget = new Thread(() -> pitchGoTo(PITCH_AUTO_SPEED,PITCH_TARGET_2,5));
                }
                pitchGoingToTarget.start();
            }
            previousGamepad2XState = gamepad2.x;

            // --------------------------- OTHER --------------------------- //

            updateHanging();

            // --------------------------- TELEMETRY --------------------------- //

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("isReverse", isReverse);
            telemetry.addData("isHanging", isHanging);
            telemetry.addData("pitchMotorAutoMoving", pitchMotorAutoMoving);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontDrive.getPower(), rightFrontDrive.getPower());
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackDrive.getPower(), rightBackDrive.getPower());
            telemetry.addData("Linear powers", "L: %4.2f, R: %4.2f", linearLeft.getPower(), linearRight.getPower());
            telemetry.addData("Linear positions", "L: %d, R: %d", linearLeft.getCurrentPosition(), linearRight.getCurrentPosition());
            telemetry.addData("Pitch", "pow: %4.2f, pos: %d", pitchMotor.getPower(), pitchMotor.getCurrentPosition());
            telemetry.addData("Actuator", "pow: %4.2f, pos: %d", actuator.getPower(), actuator.getCurrentPosition());
            telemetry.addData("Claw Pitch", "pos: %4.2f", clawPitch.getPosition());
            telemetry.addData("Claw", "pos: %4.2f", claw.getPosition());
            telemetry.update();
        }
    }

    private void openClaw() {
        claw.setPosition(CLAW_OPEN_POS);
    }

    private void closeClaw() {
        claw.setPosition(CLAW_CLOSE_POS);
    }

    private void clawPitchGoLower() {
        clawPitch.setPosition(CLAW_PITCH_LOWER);
        clawPitchPosition = CLAW_PITCH_LOWER;
    }

    private void clawPitchGoUpper() {
        clawPitch.setPosition(CLAW_PITCH_UPPER);
        clawPitchPosition = CLAW_PITCH_UPPER;
    }

    private void clawPitchGoTo(double goToPos) {
        clawPitch.setPosition(goToPos);
        clawPitchPosition = goToPos;
    }

    public void pitchGoTo(double speed, int ticks, double timeoutS) {
        if (opModeIsActive()) {
            pitchMotor.setTargetPosition(ticks);
            pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            pitchMotor.setPower(speed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (pitchMotor.isBusy())) {
                pitchMotorAutoMoving = true;
            }

            pitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pitchMotor.setPower(0);

            pitchMotorAutoMoving = false;

            pitchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private double getAppliedPower(double linearPower) {
        int leftPos = linearLeft.getCurrentPosition();
        int rightPos = linearRight.getCurrentPosition();

        // Handle the case when the user stops (linearPower == 0)
        if (linearPower == 0) {
            // Check if positions are negative and reset to 0
            if (leftPos < 0) {
                linearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (rightPos < 0) {
                linearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            return 0.105;
        }

        double leftScaling = 1.0;
        double rightScaling = 1.0;

        // Determine scaling for left motor
        if (linearPower > 0) { // Moving up, check upper region (pos > 2625)
            if (leftPos > 2625) {
                leftScaling = computeScalingFactor(leftPos, true);
            }
        } else { // Moving down, check lower region (pos < 1125)
            if (leftPos < 1125) {
                leftScaling = computeScalingFactor(leftPos, false);
            }
        }

        // Determine scaling for right motor
        if (linearPower > 0) { // Moving up
            if (rightPos > 2625) {
                rightScaling = computeScalingFactor(rightPos, true);
            }
        } else { // Moving down
            if (rightPos < 1125) {
                rightScaling = computeScalingFactor(rightPos, false);
            }
        }

        double minScaling = Math.min(leftScaling, rightScaling);

        return linearPower * minScaling;
    }

    private double computeScalingFactor(int pos, boolean isUpper) {
        final double k = Math.log(5); // Natural logarithm of 5 to achieve 0.2 at the limit
        if (isUpper) {
            // Upper region: from 2625 to 3750
            double distance = pos - 2625;
            double normalized = distance / 1125.0;
            return Math.exp(-k * normalized);
        } else {
            // Lower region: from 0 to 1125
            double distance = 1125 - pos;
            double normalized = distance / 1125.0;
            return Math.exp(-k * normalized);
        }
    }

    private void updateHanging() {
        if (gamepad2.y && !GP2aButtonPreviousState) {
            isHanging = !isHanging;
        }
        GP2aButtonPreviousState = gamepad2.y;

        if (isHanging) {
            linearLeft.setPower(-0.35);
            linearRight.setPower(-0.35);
        }
    }

    private boolean GP1aButtonPreviousState = false;
    private boolean GP2aButtonPreviousState = false;

    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}