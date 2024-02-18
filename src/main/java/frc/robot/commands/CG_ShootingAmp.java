package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TilterConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;

public class CG_ShootingAmp extends SequentialCommandGroup {
    public CG_ShootingAmp(Shooter shooter, Tilter tilter, Indexer indexer) {
        super(
                new ShooterSetpointSpeed(shooter, ShooterConstants.AMP_SPEED),
                new TilterSetpointPosition(tilter, TilterConstants.AMP_POSITION),
                new SequentialCommandGroup(
                        new ShooterTilterArmed(shooter, tilter),
                        new IndexerKick(indexer)
                )
        );
    }
}