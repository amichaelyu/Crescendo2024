package frc.robot.commands;



import frc.robot.subsystems.Shooter;


import edu.wpi.first.wpilibj2.command.Command;


public class AutoShoot extends Command {

      private final Shooter m_shooter;
      private double pwr;
      private double durationMillis;
      private long startTime;
 


  public AutoShoot(Shooter shooter, double pwr, double seconds) {

      m_shooter = shooter;
      this.pwr=pwr;
      durationMillis=seconds*1000;
      addRequirements(m_shooter);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    m_shooter.launch(pwr);

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
      m_shooter.stop();
    }
  }
}