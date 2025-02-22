package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.autoCommands.PointToPointCmd;
import org.firstinspires.ftc.teamcode.subsystem.util.Point;

@Autonomous(name = "BLUE | SPECIMEN", preselectTeleOp = "TeleOp")
public class AutoSpecimen extends AutoBase {
    private final Point specimenScorePoint = new Point(0, -32.5, 90);
    private final Point specimenPickUpPoint = new Point(40, -58.5, 90);
    @Override
    public void initialize() {
        super.initialize();
        vera.drivetrain.recalibratePinpoint();
        vera.drivetrain.setTargetPoint(INITIAL_SPECIMEN_POINT);
        vera.schedule(
                new RunCommand(vera::read),
                new RunCommand(vera::loop),
                new RunCommand(vera::write),
                new SequentialCommandGroup(
                        new PointToPointCmd.SetPoint(vera.drivetrain, INITIAL_SPECIMEN_POINT),
                        //Schedule all actions here
                )
        );
    }
}
