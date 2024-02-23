package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CG_ShootingLimeNoRot extends ParallelDeadlineGroup {
    public CG_ShootingLimeNoRot() {
        super(
            new SequentialCommandGroup(
                    new TilterLime(),
                    new ShooterTilterArmed(),
                    new IndexerKick()
            ),
            new ShooterMaxPower()
        );
    }
}