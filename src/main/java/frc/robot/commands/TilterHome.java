package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TilterConstants;
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
            System.out.println("at bottom");
            isHomed = true;
        }
        if (!isHomed) {
            System.out.println("going down");
            tilter.setVoltage(-3);
        }
        else {
            System.out.println("running PID");
            tilter.setPosition(TilterConstants.START_POSITION);
        }
    }

    @Override
    public boolean isFinished() {
        return tilter.atSetpoint() && isHomed && (tilter.getPosition() > 30);
    }

    @Override
    public void end(boolean interrupted) {
        tilter.stop();
    }
}