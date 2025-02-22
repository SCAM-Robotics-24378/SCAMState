package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.robot.Vera;
import org.firstinspires.ftc.teamcode.subsystem.util.Point;

abstract public class AutoBase extends CommandOpMode {
    private final double ROBOT_WIDTH = 17, ROBOT_LENGTH = 14, TILE_SIZE_IN = 23.75, HAlF_FIELD_LENGTH = 70.3;
    protected final Point INITIAL_SPECIMEN_POINT = new Point(TILE_SIZE_IN - (ROBOT_WIDTH / 2), (-HAlF_FIELD_LENGTH + (ROBOT_LENGTH / 2)), 90);
    protected final Point INITIAL_BUCKET_POINT = new Point(-TILE_SIZE_IN + (ROBOT_WIDTH / 2), (-HAlF_FIELD_LENGTH + (ROBOT_LENGTH / 2)), 90);
    protected Vera vera;

    @Override
    public void initialize() {
        vera = new Vera(telemetry, hardwareMap, false);
    }

    @Override
    public void reset() {
        vera.reset();
        super.reset();
    }
}