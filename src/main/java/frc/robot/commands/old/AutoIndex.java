package frc.robot.commands.old;



import frc.robot.subsystems.Indexer;


import edu.wpi.first.wpilibj2.command.Command;


public class AutoIndex extends Command {

      private final Indexer m_indexer = Indexer.getInstance();
      private double pwr;
      private double durationMillis;
      private long startTime;
 


  public AutoIndex(double pwr, double seconds) {

      this.pwr=pwr;
      durationMillis=seconds*1000;
      addRequirements(m_indexer);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    m_indexer.move(pwr);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use the
    // scheduler to end the command when the button is released.
    return System.currentTimeMillis() >startTime+durationMillis;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the wheels when the command ends.
    if (durationMillis !=  0)
    { 
      m_indexer.stop();
    }
  }
}