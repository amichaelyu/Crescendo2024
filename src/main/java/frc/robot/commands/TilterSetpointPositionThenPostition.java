package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tilter;


public class TilterSetpointPositionThenPostition extends Command {
    private final Tilter tilter = Tilter.getInstance();
    private final double position1, position2;

    public TilterSetpointPositionThenPostition(double position1, double position2) {
        addRequirements(this.tilter);
        this.position1 = position1;
        this.position2 = position2;
    }

    @Override
    public void initialize() {
        tilter.setPosition(position1);
    }

    @Override
    public void execute() {
        tilter.setPosition(position1);
    }

    @Override
    public boolean isFinished() {
        return tilter.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        tilter.setPosition(position2);
    }
}