package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="GyroTest", group="Linear OpMode")
public class GyroTest extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private IMU Gyro = null;

    public void runOpMode() {

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Gyro = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters IMUParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        Gyro.initialize(IMUParams);
        Gyro.resetYaw();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Orientation robotOrientation = Gyro.getRobotOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.RADIANS
            );
            if (gamepad1.y){
                Gyro.resetYaw();
            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("idk what quadternion is", Gyro.getRobotOrientationAsQuaternion().normalized());
            telemetry.addData("Gyro Angles", "%4.2f", robotOrientation.thirdAngle);
            double Direction1 = Math.sin(Math.PI/4  - robotOrientation.thirdAngle); // https://www.desmos.com/calculator/rqqamhfeek
            double Direction2 = Math.sin(-Math.PI/4 - robotOrientation.thirdAngle); // https://www.desmos.com/calculator/dminewe5vs
            telemetry.addData("Sins, unscaled", "%4.2f %4.2f", Direction1, Direction2);
            Direction1 *= 2;
            Direction2 *= 2;
            double max = Math.max(Math.abs(Direction1), Math.abs(Direction2));
            if (max > 1.0) {
                Direction1  /= max ;
                Direction2 /= max ;
            }
            telemetry.addData("Sins, scaled", "%4.2f %4.2f", Direction1, Direction2);
            telemetry.update();
        }
    }}