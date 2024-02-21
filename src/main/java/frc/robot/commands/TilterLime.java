package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TilterConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Tilter;


public class TilterLime extends Command {
    private final Limelight limelight = Limelight.getInstance();
    private final Tilter tilter = Tilter.getInstance();
    private double distance;

    public TilterLime() {
        addRequirements(this.tilter);
    }

    @Override
    public void initialize() {
        distance = limelight.distanceToTarget();
    }

    @Override
    public void execute() {
        tilter.setPosition(TilterConstants.tilterMap.get(distance));
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