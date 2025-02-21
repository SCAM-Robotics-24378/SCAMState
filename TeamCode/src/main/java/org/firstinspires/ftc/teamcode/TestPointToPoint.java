package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.PDFLController;

//@Disabled
@Config
@TeleOp(group = "Test", name = "TestPointToPoint")
public class TestPointToPoint extends LinearOpMode {
    public static double pX = 0.0325, dX = 0.005, fX = 0.0, lX = 0.1;
    public static double pY = 0.03, dY = 0.005, fY = 0.0, lY = 0.22;
    public static double pH = 0.6, dH = 0.005, fH = 0.0, lH = 0.13;
    public static double targX, targY, targHead;
    private final double ROBOT_WIDTH = 15.5, ROBOT_LENGTH = 16.0, TILE_SIZE_IN = 23.75, HAlF_FIELD_LENGTH = 70.3;
    private final double HEADING_TOLERANCE_RAD = Math.toRadians(1.0), X_TOLERANCE = 0.25, Y_TOLERANCE = 0.25;
    private final Pose2D startPose = new Pose2D(
            DistanceUnit.INCH,
            0,
            0,
            AngleUnit.RADIANS,
            0);
    public double xPow, yPow, headingPow;
    private Pose2D targetPose = new Pose2D
            (DistanceUnit.INCH,
                    0,
                    0,
                    AngleUnit.RADIANS,
                    0);
    private DcMotor fl, bl, fr, br;
    private PDFLController xController, yController, headingController;
    private GoBildaPinpointDriver pinpoint;
    private Telemetry telemetry;
    private double robotX_in, robotY_in, rawRobotHeading_rad;
    private double headingError_rad, correctedHeading;

    private void initialize() {
        telemetry = FtcDashboard.getInstance().getTelemetry();

        fl = hardwareMap.get(DcMotor.class, "leftFront");
        bl = hardwareMap.get(DcMotor.class, "leftRear");
        fr = hardwareMap.get(DcMotor.class, "rightFront");
        br = hardwareMap.get(DcMotor.class, "rightRear");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        xController = new PDFLController(pX, dX, fX, lX, X_TOLERANCE);
        yController = new PDFLController(pY, dY, fY, lY, Y_TOLERANCE);
        headingController = new PDFLController(pH, dH, fH, lH, HEADING_TOLERANCE_RAD);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(-90, 95);

        targX = targetPose.getX(DistanceUnit.INCH);
        targY = targetPose.getY(DistanceUnit.INCH);
        targHead = Math.toDegrees(targetPose.getHeading(AngleUnit.RADIANS));
    }

    private void read() {
        final double MM_TO_IN = 0.03937008;
        pinpoint.update();
        robotX_in = pinpoint.getPosX() * MM_TO_IN;
        robotY_in = pinpoint.getPosY() * MM_TO_IN;
        rawRobotHeading_rad = pinpoint.getHeading();
    }

    private void update() {
        // Update controller PIDs to work well with FTC Dashboard.
        xController.setPDFLConstants(pX, dX, fX, lX);
        yController.setPDFLConstants(pY, dY, fY, lY);
        headingController.setPDFLConstants(pH, dH, fH, lH);

        headingError_rad = AngleUnit.normalizeRadians(targetPose.getHeading(AngleUnit.RADIANS) - rawRobotHeading_rad);
        double xErr_in = targetPose.getX(DistanceUnit.INCH) - robotX_in;
        double yErr_in = targetPose.getY(DistanceUnit.INCH) - robotY_in;
        headingPow = -headingController.calculate(headingError_rad);
        correctedHeading = targetPose.getHeading(AngleUnit.RADIANS) - headingError_rad;
        double xRotated_in = xErr_in * Math.cos(-correctedHeading) - yErr_in * Math.sin(-correctedHeading);
        double yRotated_in = xErr_in * Math.sin(-correctedHeading) + yErr_in * Math.cos(-correctedHeading);
        telemetry.addData("yErr", yErr_in);
        telemetry.addData("xErrRotated", xRotated_in);
        telemetry.addData("xErr", xErr_in);
        telemetry.addData("yErrRotated", yRotated_in);
        xPow = xController.calculate(xRotated_in);
        yPow = - yController.calculate(yRotated_in);
        targetPose = new Pose2D(DistanceUnit.INCH, targX, targY, AngleUnit.RADIANS, Math.toRadians(targHead));
    }

    private void write() {
        // x, y, theta input mixing
        if (gamepad1.right_trigger >= 0.05) {
            fl.setPower((xPow + yPow + headingPow) * gamepad1.right_trigger);
            bl.setPower((xPow - yPow + headingPow) * gamepad1.right_trigger);
            fr.setPower((xPow - yPow - headingPow) * gamepad1.right_trigger);
            br.setPower((xPow + yPow - headingPow) * gamepad1.right_trigger);
        } else if (gamepad1.a) {
            fl.setPower(0.5);
        } else if (gamepad1.b) {
            bl.setPower(0.5);
        } else if (gamepad1.x) {
            fr.setPower(0.5);
        } else if (gamepad1.y) {
            br.setPower(0.5);
        } else {
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        }

        telemetry.addData("Robot X", robotX_in);
        telemetry.addData("Robot Y", robotY_in);
        telemetry.addData("robotHeading", Math.toDegrees(correctedHeading));
        telemetry.addData("Target X", targetPose.getX(DistanceUnit.INCH));
        telemetry.addData("Target Y", targetPose.getY(DistanceUnit.INCH));
        telemetry.addData("Target Heading", targetPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("HeadingPow", headingPow);
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        pinpoint.resetPosAndIMU();
        sleep(300);
        pinpoint.setPosition(startPose);
        while (opModeIsActive() && !isStopRequested()) {
            read();
            update();
            write();
            sleep(20);
        }
    }
}