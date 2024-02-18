package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;


public class ShooterLime extends Command {
    private final Shooter shooter;
    private final Limelight limelight;
    private double distance;

    public ShooterLime(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        addRequirements(this.shooter, this.limelight);
    }

    @Override
    public void initialize() {
        distance = limelight.distanceToTarget();
    }

    @Override
    public void execute() {
        shooter.setSpeed(ShooterConstants.shooterMap.get(distance));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}