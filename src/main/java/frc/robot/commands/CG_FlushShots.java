package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class CG_FlushShots extends ParallelCommandGroup {
    public CG_FlushShots() {
        super(
                new IntakeReverse(),
                new IndexerFlush()
        );
    }
}