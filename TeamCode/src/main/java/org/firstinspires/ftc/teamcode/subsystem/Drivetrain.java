package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.PDFLController;
import org.firstinspires.ftc.teamcode.robot.Vera;
import org.firstinspires.ftc.teamcode.subsystem.util.Point;

import java.util.function.DoubleSupplier;

@Config
public class Drivetrain extends SubsystemBase {

    // Adjustment constants for "precise navigation mode"; applied when the thrust control
    // (left stick for drone controls) is pushed past a threshold.
    private static final double PRECISE_NAV_THRUST_THRESH = 0.5; // Stick more than half forward
    private static final double PRECISE_NAV_SLOW_FACTOR = 0.65;

    // Factor to reduce overall sensitivity in teleop.
    private static final double TELEOP_POWER_FACTOR = 1.0;
    private final double pX = 0.033, dX = 0.005, fX = 0.0, lX = 0.1;
    private final double pY = 0.04, dY = 0.005, fY = 0.0, lY = 0.22;
    private final double pH = 0.63, dH = 0.005, fH = 0.0, lH = 0.13;
    private final double HEADING_TOLERANCE_RAD = Math.toRadians(0.5), X_TOLERANCE = 0.15, Y_TOLERANCE = 0.15;
    private final double SPECIMEN_SCORE_OFFSET_IN = 3.0;
    // Thresholds for verifying dt has reached its target
    private final double POSITION_THRESH_IN = 2; //TODO: Find val
    private final double HEADING_THRESH_DEG = 5; //TODO: Find val
    private final double POSITION_CONFIRMATION_SEC = 0.25; //TODO: Find val
    private final double MAX_ALLOWED_DRIVE_SEC = 3; //TODO: Verify
    private final double STICK_THRESH = 0.05;

    private final Motor leftFront;
    private final Motor leftBack;
    private final Motor rightBack;
    private final Motor rightFront;
    private final boolean DISPLAY_TELEM = false;
    private final PDFLController xController;
    private final PDFLController yController;
    private final PDFLController headingController;
    private final GoBildaPinpointDriver pinpoint;
    private final Telemetry telemetry;
    public double xPow, yPow, headingPow;
    ElapsedTime targetTimer = new ElapsedTime();
    ElapsedTime overtimeProtection = new ElapsedTime();
    private GamepadEx gp1 = null;
    private DoubleSupplier g1RightStickY = null;
    private DoubleSupplier g1RightStickX = null;
    private DoubleSupplier g1LeftStickX = null;
    private DoubleSupplier g1LeftStickY = null;
    private boolean isAutoDrive = false;
    private double drive = 0;
    private double turn = 0;
    private double strafe = 0;
    private double thrust = 0;
    private double prev_leftFrontPower = -999;
    private double prev_leftBackPower = -999;
    private double prev_rightBackPower = -999;
    private double prev_rightFrontPower = -999;
    private double leftFrontPower, leftBackPower, rightBackPower, rightFrontPower;
    private double teleHeadingTarget = 0.0;
    private double robotX_in, robotY_in, rawRobotHeading_rad, correctedHeading_rad;
    private boolean isDtAtTarget;
    private final Point targetPoint = new Point(0, 0, 0);
    private double headingError_rad;

    public Drivetrain(Motor leftFront, Motor leftBack, Motor rightBack, Motor rightFront,
                      GoBildaPinpointDriver pinpoint,
                      Telemetry telemetry) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.rightFront = rightFront;

        this.leftFront.setInverted(true);
        this.leftBack.setInverted(true);
        this.rightBack.setInverted(false);
        this.rightFront.setInverted(false);

        this.leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.xController = new PDFLController(pX, dX, fX, lX, X_TOLERANCE);
        this.yController = new PDFLController(pY, dY, fY, lY, Y_TOLERANCE);
        this.headingController = new PDFLController(pH, dH, fH, lH, HEADING_TOLERANCE_RAD);

        this.pinpoint = pinpoint;
        this.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        this.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        this.pinpoint.setOffsets(-90, 95);

        if (Vera.isAutonomous) isAutoDrive = true;

        this.telemetry = telemetry;
    }

    public Drivetrain(Motor leftFront, Motor leftBack, Motor rightBack, Motor rightFront,
                      GoBildaPinpointDriver pinpoint,
                      Telemetry telemetry, GamepadEx gamepad1) {
        this(leftFront, leftBack, rightBack, rightFront, pinpoint, telemetry);
        // Bind stick inputs to drone-mode control variables.
        this.gp1 = gamepad1;
        if (!Vera.isAutonomous) {
            g1RightStickY = gp1::getRightY;
            g1RightStickX = gp1::getRightX;
            g1LeftStickX = gp1::getLeftX;
            g1LeftStickY = gp1::getLeftY;
        }
    }

    public void recalibratePinpoint() {
        pinpoint.recalibrateIMU();
    }

    public void setCurrentPoint(Point point) {
        pinpoint.setPosition(point.toPose());
        targetPoint.setPoint(point.x(), point.y(), Math.toRadians(point.heading()));
    }

    public void setTargetPoint(Point point) {
        targetPoint.setPoint(point.x(), point.y(), Math.toRadians(point.heading()));
    }

    public void setTargetHeading(double heading_deg) {
        targetPoint.setHeading(Math.toRadians(heading_deg));
    }

    public void setIsAutoDriveFalse() {
        isAutoDrive = false;
    }

    public void setIsAutoDriveTrue() {
        isAutoDrive = true;
    }

    public void setTargetVec(double x, double y) {
        targetPoint.setX(x);
        targetPoint.setY(y);
    }

    public void setDriveTarget(double distance_in) {
        targetPoint.setX(targetPoint.x() + distance_in * Math.cos(targetPoint.heading()));
        targetPoint.setY(targetPoint.y() + distance_in * Math.sin(targetPoint.heading()));
    }

    public void setIsAtTargetFalse() {
        isDtAtTarget = false;
        overtimeProtection.reset();
        targetTimer.reset();
    }

    public void setStrafeTarget(double distance_in) {
        double perpHeading = targetPoint.heading() + (Math.PI / 2);
        double newX = targetPoint.x() + distance_in * Math.cos(perpHeading);
        double newY = targetPoint.y() + distance_in * Math.sin(perpHeading);
        targetPoint.setX(newX);
        targetPoint.setY(newY);
    }

    public void setXTarget(double x_in) {
        targetPoint.setX(x_in);
    }

    public void setYTarget(double y_in) {
        targetPoint.setY(y_in);
    }

    public boolean isAtTarget() {
        double xDelta = (targetPoint.x() - robotX_in);
        double yDelta = (targetPoint.y() - robotY_in);
        double distanceSquared = xDelta * xDelta + yDelta * yDelta;
        if (distanceSquared >= POSITION_THRESH_IN * POSITION_THRESH_IN
                || Math.toDegrees(Math.abs(headingError_rad)) >= HEADING_THRESH_DEG) {
            targetTimer.reset();
        }
        isDtAtTarget = (targetTimer.seconds() >= POSITION_CONFIRMATION_SEC)
                || (overtimeProtection.seconds() >= MAX_ALLOWED_DRIVE_SEC);
        return isDtAtTarget;
    }

    private boolean isRightStickOutOfThreshold() {
        return (Math.abs(drive) >= STICK_THRESH || Math.abs(strafe) >= STICK_THRESH);
    }

    private boolean isLeftStickOutOfThreshold() {
        return (Math.abs(turn) >= STICK_THRESH || Math.abs(thrust) >= STICK_THRESH);
    }

    private void translateDroneFlightControls() {
        drive = drive * (1.0 - lX) + (Math.signum(drive) * lX);
        if (isLeftStickOutOfThreshold()) turn = turn *  (1.0 - lH) + (Math.signum(turn) * lX);
        strafe = strafe * (1.0 - lY) + (Math.signum(strafe) * lX);
        // Mix the stick commands into power for each wheel and apply an overall scale factor.
        leftFrontPower = TELEOP_POWER_FACTOR *
                -(drive - turn - strafe);
        leftBackPower = TELEOP_POWER_FACTOR *
                -(drive - turn + strafe);
        rightBackPower = TELEOP_POWER_FACTOR *
                -(drive + turn - strafe);
        rightFrontPower = TELEOP_POWER_FACTOR *
                -(drive + turn + strafe);
        // If the thrust command exceeds a threshold (e.g., left stick is more than half-way
        // forward), then scale the power down further for "precise navigation mode".
        if (thrust > PRECISE_NAV_THRUST_THRESH) {
            leftFrontPower *= PRECISE_NAV_SLOW_FACTOR;
            leftBackPower *= PRECISE_NAV_SLOW_FACTOR;
            rightBackPower *= PRECISE_NAV_SLOW_FACTOR;
            rightFrontPower *= PRECISE_NAV_SLOW_FACTOR;
        }
    }

    private void normalizePowers() {
        // Normalize motor powers so no magnitude is greater than 1.
        double maxPower = Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                Math.max(Math.abs(rightBackPower), Math.abs(rightFrontPower)));
        if (maxPower > 1) {
            leftFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
            rightFrontPower /= maxPower;
        }
    }

    private void driveToTarget() {
        headingError_rad = AngleUnit.normalizeRadians(targetPoint.heading() - rawRobotHeading_rad);
        double xErr_in = targetPoint.x() - robotX_in;
        double yErr_in = targetPoint.y() - robotY_in;

        headingPow = -headingController.calculate(headingError_rad);
        correctedHeading_rad = targetPoint.heading() - headingError_rad;
        double xErrorRotated_in = xErr_in * Math.cos(-correctedHeading_rad) - yErr_in * Math.sin(-correctedHeading_rad);
        double yErrorRotated_in = xErr_in * Math.sin(-correctedHeading_rad) + yErr_in * Math.cos(-correctedHeading_rad);
        xPow = xController.calculate(xErrorRotated_in);
        yPow = -yController.calculate(yErrorRotated_in);

        leftFrontPower = xPow + yPow + headingPow;
        leftBackPower = xPow - yPow + headingPow;
        rightFrontPower = xPow - yPow - headingPow;
        rightBackPower = xPow + yPow - headingPow;
    }

    public void read() {
        if (!Vera.isAutonomous) {
            drive = g1RightStickY.getAsDouble();
            strafe = g1RightStickX.getAsDouble();
            turn = g1LeftStickX.getAsDouble();
//            thrust = Math.abs(g1LeftStickY.getAsDouble());
            thrust = 0;
        }
        final double MM_TO_IN = 0.03937008;
        pinpoint.update();
        robotX_in = pinpoint.getPosX() * MM_TO_IN;
        robotY_in = pinpoint.getPosY() * MM_TO_IN;
        rawRobotHeading_rad = pinpoint.getHeading();
    }

    public void loop() {
        if (Vera.isAutonomous) {
            driveToTarget();
            normalizePowers();
        } else if (isRightStickOutOfThreshold() || isLeftStickOutOfThreshold()) {
            translateDroneFlightControls();
            normalizePowers();
        } else {
            leftFrontPower = 0.0;
            leftBackPower = 0.0;
            rightFrontPower = 0.0;
            rightBackPower = 0.0;
        }
    }

    public void write() {
        if (leftFrontPower != prev_leftFrontPower) {
            leftFront.set(leftFrontPower);
            prev_leftFrontPower = leftFrontPower;
        }
        if (leftBackPower != prev_leftBackPower) {
            leftBack.set(leftBackPower);
            prev_leftBackPower = leftBackPower;
        }
        if (rightBackPower != prev_rightBackPower) {
            rightBack.set(rightBackPower);
            prev_rightBackPower = rightBackPower;
        }
        if (rightFrontPower != prev_rightFrontPower) {
            rightFront.set(rightFrontPower);
            prev_rightFrontPower = rightFrontPower;
        }
        if (DISPLAY_TELEM) {
            telemetry.addData("X", robotX_in);
            telemetry.addData("Y", robotY_in);
            telemetry.addData("Heading", Math.toDegrees(correctedHeading_rad));
            telemetry.addData("xTarg", targetPoint.x());
            telemetry.addData("yTarg", targetPoint.y());
            telemetry.addData("headingTarg", targetPoint.heading());
        }
    }

    private enum DriveState {
        START,
        LEG1,
        LEG2,
    }
}