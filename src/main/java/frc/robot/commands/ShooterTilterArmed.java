package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;


public class ShooterTilterArmed extends Command {
    private final Shooter shooter = Shooter.getInstance();
    private final Tilter tilter = Tilter.getInstance();

    public ShooterTilterArmed() {}

    @Override
    public boolean isFinished() {
        return tilter.atSetpoint() && shooter.atSetpoint();
    }
}