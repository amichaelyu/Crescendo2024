package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class CG_IntakeIndexer extends ParallelCommandGroup {
    public CG_IntakeIndexer() {
        super(
                new IntakeRun(),
                new IndexerRun()
        );
    }
}