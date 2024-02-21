package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tilter;


public class TilterHome extends Command {
    private final Tilter tilter = Tilter.getInstance();
    private boolean isHomed = false;

    public TilterHome() {
        addRequirements(this.tilter);
    }

    @Override
    public void initialize() {
        isHomed = false;
    }

    @Override
    public void execute() {
        if (tilter.isAtBottom()) {
            isHomed = true;
        }
        if (!isHomed) {
            tilter.setVoltage(-3);
        }
        else {
//            tilter.setPosition();
        }
    }

    @Override
    public boolean isFinished() {
        return !tilter.isAtBottom();
    }

    @Override
    public void end(boolean interrupted) {
        tilter.stop();
        tilter.homed();
    }
}