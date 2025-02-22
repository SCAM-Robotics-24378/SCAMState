package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Vera;

@TeleOp(name = "TeleOp")
public class Tele extends CommandOpMode {
    Vera vera;

    @Override
    public void initialize() {
        vera = new Vera(
                telemetry,
                hardwareMap,
                true,
                new GamepadEx(gamepad1),
                new GamepadEx(gamepad2));

        schedule(
                new RunCommand(vera::read),
                new RunCommand(vera::loop),
                new RunCommand(vera::write)
        );

        vera.drivetrain.recalibratePinpoint();

    }

    @Override
    public void reset() {
        vera.reset();
        super.reset();
    }
}
