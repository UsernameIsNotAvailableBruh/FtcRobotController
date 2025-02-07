package pedroPathing.constants;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.constants.PinpointConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "leftFrontDrive";
        FollowerConstants.leftRearMotorName = "leftBackDrive";
        FollowerConstants.rightFrontMotorName = "rightFrontDrive";
        FollowerConstants.rightRearMotorName = "rightBackDrive";

        FollowerConstants.leftFrontMotorDirection = DcMotor.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotor.Direction.FORWARD;
        FollowerConstants.rightFrontMotorDirection = DcMotor.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotor.Direction.FORWARD;

        FollowerConstants.mass = 13.1; //kg

        FollowerConstants.xMovement = (59.5724+59.9676+58.506+58.8006)/4;
        FollowerConstants.yMovement = (50.2009+48.8625+49.2683+49.4428)/4;

        FollowerConstants.forwardZeroPowerAcceleration = (-30.7856-28.8964-29.062-31.554)/4;
        FollowerConstants.lateralZeroPowerAcceleration = (-65.9526-59.5283-54.4454-59.5369)/4;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.15,0,.01,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.5,0,0.02,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(.015,0,.000001,0.6,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 3; //2 to 6
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}