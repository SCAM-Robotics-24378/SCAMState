package org.firstinspires.ftc.teamcode.opmodes.auto.autoCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.util.Point;

public class PointToPointCmd {
    public static class DriveCommand extends CommandBase {
        private final Drivetrain drivetrain;
        private final double distance;

        public DriveCommand(Drivetrain drivetrain, double distance) {
            this.drivetrain = drivetrain;
            addRequirements(drivetrain);
            this.distance = distance;
        }

        @Override
        public void initialize() {
            drivetrain.setIsAtTargetFalse();
            drivetrain.setDriveTarget(distance);
        }

        @Override
        public boolean isFinished() {
            return drivetrain.isAtTarget();
        }
    }

    public static class StrafeCommand extends CommandBase {
        private final Drivetrain drivetrain;
        private final double strafeDist;

        public StrafeCommand(Drivetrain drivetrain, double strafeDist) {
            this.drivetrain = drivetrain;
            addRequirements(drivetrain);
            this.strafeDist = strafeDist;
        }

        @Override
        public void initialize() {
            drivetrain.setIsAtTargetFalse();
            drivetrain.setStrafeTarget(strafeDist);
        }

        @Override
        public boolean isFinished() {
            return drivetrain.isAtTarget();
        }
    }

    public static class DriveToPointCommand extends CommandBase {
        private final Drivetrain drivetrain;
        private final Point targetPoint;

        public DriveToPointCommand(Drivetrain drivetrain, Point targetPoint) {
            this.drivetrain = drivetrain;
            addRequirements(drivetrain);
            this.targetPoint = targetPoint;
        }

        @Override
        public void initialize() {
            drivetrain.setIsAtTargetFalse();
            drivetrain.setTargetPoint(targetPoint);
        }

        @Override
        public boolean isFinished() {
            return drivetrain.isAtTarget();
        }
    }

    public static class DriveToXCommand extends CommandBase {
        private final Drivetrain drivetrain;
        private final double x;

        public DriveToXCommand(Drivetrain drivetrain, double x) {
            this.drivetrain = drivetrain;
            addRequirements(drivetrain);
            this.x = x;
        }

        @Override
        public void initialize() {
            drivetrain.setIsAtTargetFalse();
            drivetrain.setXTarget(x);
        }

        @Override
        public boolean isFinished() {
            return drivetrain.isAtTarget();
        }
    }

    public static class DriveToYCommand extends CommandBase {
        private final Drivetrain drivetrain;
        private final double y;

        public DriveToYCommand(Drivetrain drivetrain, double y) {
            this.drivetrain = drivetrain;
            addRequirements(drivetrain);
            this.y = y;
        }

        @Override
        public void initialize() {
            drivetrain.setIsAtTargetFalse();
            drivetrain.setYTarget(y);
        }

        @Override
        public boolean isFinished() {
            return drivetrain.isAtTarget();
        }
    }

    public static class TurnToHeadingCommand extends CommandBase {
        private final Drivetrain drivetrain;
        private final double heading;

        public TurnToHeadingCommand(Drivetrain drivetrain, double heading_deg) {
            this.drivetrain = drivetrain;
            addRequirements(drivetrain);
            this.heading = heading_deg;
        }

        @Override
        public void initialize() {
            drivetrain.setIsAtTargetFalse();
            drivetrain.setTargetHeading(heading);
        }

        @Override
        public boolean isFinished() {
            return drivetrain.isAtTarget();
        }
    }

    public static class SetPoint extends CommandBase {
        private final Drivetrain drivetrain;
        private final Point point;
        public SetPoint(Drivetrain drivetrain, Point point) {
            this.drivetrain = drivetrain;
            addRequirements(drivetrain);
            this.point = point;
        }

        @Override
        public void initialize() {
            drivetrain.setCurrentPoint(point);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}
