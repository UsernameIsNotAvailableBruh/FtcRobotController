package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;
import java.util.EnumMap;
import java.util.List;
import java.util.Random;

@Config
@TeleOp(name="PIDTest", group="Linear OpMode")
public class PIDTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private Servo pitchServo = null;
    private DcMotorEx pitchMotor = null;
    private DcMotorEx acturio = null;


    //PIDF variables Configs
    public static volatile boolean UsePID = true;
    public static volatile double Kp = .01; //highest is .055 (not recommended), lowest is .001 (slow but nice)
    public static volatile double Ki = .042; //honestly idk. too high means lost of precision& more power is used and too low needs a huge isumlimit and more time
    public static volatile double integralSumLimit = 3; //the final isum should not go over/under 1/-1... but just in case.. so its 3 :)
    //public static volatile double Kd = 0;
    //public static volatile double Kda = 1;
    //public static volatile double Kf = 0;
    public static volatile int targetPos = 0;
    public static volatile int actuatorPos = 0;
    public static volatile boolean teleActurio = true;
    public static volatile boolean teleServos = true;
    public static          boolean randomTargetPos = false; //just pretend this is volatile like the others lol

    private double lastError = 0;

    private double lastFilterEstimate = 0;
    private double lastTarget = 0;

    ElapsedTime PIDtime = new ElapsedTime();
    private final double PPR5203_117RPM = 1425.05923;
    private double integralSum = 0;

    // Configs
    public static double pitchServoPos = .35;
    /*
     * .53 is basically 90deg
     * .85 is basically 180 deg
     * .34/.33 is as close to the pitchmotor as possible
     *  .7 is 180 parallel to robot
     */
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");

        pitchServo = hardwareMap.get(Servo.class, "clawPitch");
        pitchMotor = hardwareMap.get(DcMotorEx.class, "pitchMothy");
        acturio = hardwareMap.get(DcMotorEx.class, "actuator");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pitchMotor.setDirection(DcMotor.Direction.REVERSE);
        acturio.setDirection(DcMotor.Direction.FORWARD);
        pitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        acturio.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        acturio.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        acturio.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Wait for the game to start (driver presses START)

        //just for funsies
        double sampleRate = 0;
        double lastRuntime = 0;
        double highestSR = 0;
        double lowestSR = 100000000; //idk

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Buttons ButtonMonitorGP1 = new Buttons(false);
        Buttons ButtonMonitorGP2 = new Buttons(true);

        double telePitchServoPos = pitchServoPos;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Will run one bulk read per cycle,
            // even as frontLeftMotor.getCurrentPosition() is called twice
            // because the caches are being handled manually and cleared
            // once a loop
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            //ButtonMonitorGP1.update(); //not really used fr
            ButtonMonitorGP2.update();

            //GPAD 1:
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.
            /*leftFrontPower  = gamepad1.cross ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.circle ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.triangle ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.square ? 1.0 : 0.0;  // B gamepad*/

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

            //GPAD 2:
            if (ButtonMonitorGP2.wasPressed(buttonName.guide)){
                resetEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (teleActurio) {
                acturio.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (ButtonMonitorGP2.isPressedCurrently(buttonName.dpad_up)) {
                    acturio.setPower(.5);
                }
                else if (ButtonMonitorGP2.isPressedCurrently(buttonName.dpad_down)) {
                    acturio.setPower(-.5);
                }
                else {
                    acturio.setPower(0);
                }
            }
            else { //not sure what the max acturio pos is so prob wont use this
                acturio.setTargetPosition(actuatorPos);
                acturio.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                acturio.setPower(.1);
            }

            if (teleServos){
                if (ButtonMonitorGP2.wasPressed(buttonName.triangle)){
                    telePitchServoPos = telePitchServoPos==1?telePitchServoPos:(telePitchServoPos+.01);
                } else if (ButtonMonitorGP2.wasPressed(buttonName.cross)){
                    telePitchServoPos = telePitchServoPos==0?telePitchServoPos:(telePitchServoPos-.01);
                }
                if (telePitchServoPos>1 || telePitchServoPos<0){
                    telePitchServoPos = telePitchServoPos>1? 1:0;
                }
                //TODO: add claw servo (maybe)
                pitchServo.setPosition(telePitchServoPos);
                telemetry.addData("pitchServoPos", telePitchServoPos);
            }
            else {
                pitchServo.setPosition(pitchServoPos);
                telemetry.addData("pitchServoPos", pitchServoPos);
            }

            int currentPos = pitchMotor.getCurrentPosition();
            if (randomTargetPos && ButtonMonitorGP2.wasPressed(buttonName.circle)) {
                Random rand = new Random();
                if (currentPos > 4250/2){
                    targetPos = rand.nextInt(4250/2-100)+100;
                }
                else {
                    targetPos = rand.nextInt(4250-(4250/2))+4250/2;
                }
            } // the else is FTC Dashboard configs (randomTargetPos is false)
            double power = PitchPosPIupdate(targetPos, currentPos);
            telemetry.addData("Target pos: (pitchMotor)", targetPos);
            if (UsePID) {
                pitchMotor.setPower(power > 1 ? 1 : (power < -1 ? -1 : power));
            }
            else {
                pitchMotor.setPower(0);
            }
            telemetry.addData("Power: (pitchMotor)", pitchMotor.getPower());
            telemetry.addData("Current pos: (pitchMotor)", currentPos);
            double currentRuntime = runtime.seconds();
            sampleRate = 1/(currentRuntime-lastRuntime);
            telemetry.addData("Sample Rate Info", "%4.2f %4.2f %4.2f", sampleRate, highestSR, lowestSR);
            telemetry.addData("randomTargetPos", randomTargetPos);
            telemetry.update();
            lastRuntime = currentRuntime;
            if (sampleRate > highestSR){
                highestSR = sampleRate;
            }
            if (sampleRate < lowestSR){
                lowestSR = sampleRate;
            }
        }
    }

    public double PitchPosPIupdate(int target, int current){
        double error = target-current;
        double seconds = PIDtime.seconds();
        double p = Kp * error;
        integralSum += error*seconds;
        double i = Ki * integralSum;

        // ----------------optimizations----------------
        //integralSum optimizations:
        if (lastTarget != target) { //target pos changes, reset isum fr
            integralSum = 0;
        }
        //set limits for isum
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }
        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }
        //derivative low pass filter:
        //filter out high frequency noise to increase derivative performance
        /*double currentFilterEstimate = (Kda * lastFilterEstimate) + (1-Kda) * (error - lastError);
        lastFilterEstimate = currentFilterEstimate;*/
        // ----------------optimizations----------------

        //double derivative =  currentFilterEstimate / seconds;
        //double d = Kd * derivative;
        //In my testing, the D seems to have no positive effect. So no D.
        //double NoFilterD = (error - lastError) / seconds;
        //double f = Kf * Math.cos(current*Math.PI*2/PPR5203_117RPM); //cos(Ticks*(2pi/PPR)) //cosine to make it nonlinear, hopefully helps with nonlinear gravity stuff
        //idk if the f line ^ works or not and PID(no F) seems to work fine.

        PIDtime.reset();
        lastError = error;
        lastTarget = target;

        double power = p+i;//+d;
        telemetry.addData("Power: (PID)", power);
        if ((power < .01 && power > 0) || (power > -.01 && power < 0)) { //basically telling the PID to put the fries in the bag
            //really small values are basically zero.. so its just better to just zero em
            power = 0;
        }
        int n = 3;
        power = Math.round(power * Math.pow(10, n))/Math.pow(10, n); //round to n decimal places
        telemetry.addData("P", p);
        telemetry.addData("I", i);
        /*telemetry.addData("D", d);
        telemetry.addData("NoFilterD", NoFilterD);*/
        telemetry.addData("integralSum", integralSum);
        return power;
    }

    public void resetEncoders(DcMotor.RunMode NewMode){
        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        acturio.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pitchMotor.setMode(NewMode);
        acturio.setMode(NewMode);
        leftFrontDrive.setMode(NewMode);
        leftBackDrive.setMode(NewMode);
        rightFrontDrive.setMode(NewMode);
        rightBackDrive.setMode(NewMode);
    }

    // This button status thing is inspired by u/m0stlyharmless_user and u/fusionforscience on reddit from a post 8y ago :)
    // also ryan if youre reading this i did NOT use ai for this, this is indeed my code believe it or not. I use used the links below to help out with the logic fr
    // https://www.reddit.com/r/FTC/comments/5lpaai/comment/dbye175/?utm_source=share&utm_medium=web3x&utm_name=web3xcss&utm_term=1&utm_content=share_button
    // https://www.reddit.com/r/FTC/comments/5lpaai/comment/dcerspj/?utm_source=share&utm_medium=web3x&utm_name=web3xcss&utm_term=1&utm_content=share_button
    enum Status {
        notPressedYet,
        currentlyPressed,
        wasPressed
    }
    enum buttonName { //this and the buttonList can be optimized by only including buttons used.
        options,
        triangle,
        share,
        cross,
        square,
        circle,
        left_stick_button,
        right_stick_button,
        dpad_left,
        dpad_right,
        dpad_up,
        dpad_down,
        right_bumper,
        left_bumper,
        guide
    }
    enum Gpads { //GP stands for gamepad
        GP1,
        GP2
    }
    private class Buttons { //To add new buttons, just append it to the bottom of buttonName & append to buttonList.
        public  final EnumMap<buttonName, Status> buttonMap     = new EnumMap<>(buttonName.class);
        private final EnumMap<buttonName, Button> ButtonStorage = new EnumMap<>(buttonName.class);
        private Gpads GP;
        private final Gamepad gpad = new Gamepad();

        public Buttons(boolean isGamepad2) {
            GP = Gpads.GP1;
            if (isGamepad2) {
                GP = Gpads.GP2;
            }
            for (buttonName button : buttonName.values()) {
                ButtonStorage.put(button, new Button());
            }
        }

        public boolean wasPressedOrisPressed(buttonName button) {
            return buttonMap.get(button) == Status.wasPressed || buttonMap.get(button) == Status.currentlyPressed;
        }

        public boolean wasPressed(buttonName button) {
            return buttonMap.get(button) == Status.wasPressed;
        }

        public boolean isPressedCurrently(buttonName button) {
            return buttonMap.get(button) == Status.currentlyPressed;
        }

        public boolean NotPressed(buttonName button){
            return buttonMap.get(button) == Status.notPressedYet;
        }

        public void update(){
            if (GP == Gpads.GP1) {
                gpad.copy(gamepad1);
            }
            else {
                gpad.copy(gamepad2);
            }
            boolean[] buttonList = new boolean[]{gpad.options, gpad.triangle, gpad.share, gpad.cross, gpad.square, gpad.circle, gpad.left_stick_button, gpad.right_stick_button, gpad.dpad_left, gpad.dpad_right, gpad.dpad_up, gpad.dpad_down, gpad.right_bumper, gpad.left_bumper, gpad.guide};
            buttonName[] ButtonArr = buttonName.values();
            for (int i = 0; i< buttonList.length; i++) {
                buttonMap.put(ButtonArr[i], ButtonStorage.get(ButtonArr[i]).ButtonStatus(buttonList[i]));
            }
        }

        private class Button { //class in a class in a class for funsies
            private Status status = Status.notPressedYet;
            public Status ButtonStatus(boolean button) {
                if ( (button && status == Status.notPressedYet))
                    status = Status.currentlyPressed;
                else if (!button && status == Status.currentlyPressed)
                    status = Status.wasPressed;
                else if (status == Status.wasPressed)
                    status = Status.notPressedYet;
                return status;
            }
        }
    }
}
