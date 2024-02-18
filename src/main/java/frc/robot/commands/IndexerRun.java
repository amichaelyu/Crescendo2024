package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;


public class IndexerRun extends Command {
    private final Indexer indexer;

    public IndexerRun(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(this.indexer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        indexer.move(IndexerConstants.INDEXER_DUTY_CYCLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}