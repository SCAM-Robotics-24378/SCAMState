package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.autoCommands.PointToPointCmd;
import org.firstinspires.ftc.teamcode.robot.Vera;
import org.firstinspires.ftc.teamcode.subsystem.util.Point;

@Autonomous(name = "BLUE | BUCKET", preselectTeleOp = "TeleOp")
public class AutoBucket extends AutoBase {

    private final Point specimenScorePoint = new Point(-5, -31, 90);

    @Override
    public void initialize() {
        super.initialize();
        vera.drivetrain.recalibratePinpoint();
        vera.drivetrain.setTargetPoint(INITIAL_BUCKET_POINT);
        vera.schedule(
                new RunCommand(vera::read),
                new RunCommand(vera::loop),
                new RunCommand(vera::write),
                new SequentialCommandGroup(
                        new PointToPointCmd.SetPoint(vera.drivetrain, INITIAL_BUCKET_POINT)
                        //Schedule all actions here
                )
        );
    }
    // No run method is needed as the command op mode does all this in the background including reset (in theory)
}
