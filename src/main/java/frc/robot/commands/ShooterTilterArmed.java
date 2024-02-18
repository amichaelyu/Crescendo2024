package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;


public class ShooterTilterArmed extends Command {
    private final Shooter shooter;
    private final Tilter tilter;

    public ShooterTilterArmed(Shooter shooter, Tilter tilter) {
        this.shooter = shooter;
        this.tilter = tilter;
        addRequirements(this.shooter, this.tilter);
    }

    @Override
    public boolean isFinished() {
        return tilter.atSetpoint() && shooter.atSetpoint();
    }
}