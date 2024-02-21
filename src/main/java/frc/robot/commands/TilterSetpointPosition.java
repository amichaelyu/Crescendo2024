package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tilter;


public class TilterSetpointPosition extends Command {
    private final Tilter tilter = Tilter.getInstance();
    private final double position;

    public TilterSetpointPosition(double position) {
        addRequirements(this.tilter);
        this.position = position;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        tilter.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        return tilter.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        tilter.stop();
    }
}