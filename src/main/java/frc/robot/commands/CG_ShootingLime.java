package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class CG_ShootingLime extends ParallelCommandGroup {
    public CG_ShootingLime(Swerve swerve, Limelight limelight, Shooter shooter, Tilter tilter, Indexer indexer) {
        super(
            new SwerveRotateLime(swerve, limelight),
            new TilterLime(tilter, limelight),
            new ShooterLime(shooter, limelight),
            new SequentialCommandGroup(
                    new ShooterTilterArmed(shooter, tilter),
                    new IndexerKick(indexer)
            )
        );
    }
}