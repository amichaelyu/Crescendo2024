package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CG_ShootingLimeOnTheFly extends ParallelDeadlineGroup {
    public CG_ShootingLimeOnTheFly() {
        super(
            new SequentialCommandGroup(
                    new ShooterTilterArmed(),
                    new IndexerKick()
            ),
            new SwerveRotateLimeOnTheFly(),
            new TilterLime(),
            new ShooterMaxPower()
        );
    }
}