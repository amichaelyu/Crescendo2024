package frc.robot.subsystems;

import static frc.robot.Constants.*;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.GenericHID;


public class Shooter extends SubsystemBase {
  private final TalonFX m_rightShooter = new TalonFX(rightShooterID);
  private final TalonFX m_leftShooter = new TalonFX(leftShooterID);
  private GenericHID xbox; 
  /** Creates a new Shooter. */
  public Shooter() {
    

    m_rightShooter.getConfigurator().apply(new TalonFXConfiguration());
    m_leftShooter.getConfigurator().apply(new TalonFXConfiguration());

    m_rightShooter.setInverted(true);
    m_leftShooter.setInverted(false);
    xbox = RobotContainer.codriver.getHID();
    
  }

  public Command getIntakeCommand(){

    return this.startEnd(
      () -> {launch(kShooterIntakeSpeed);
      },
      () -> {
        stop();
      });
  }
  
  public void launch(double pwr) {
    if (xbox.getRawButton(2))
    {
      pwr=pwr*0.5;
    }
    else if (xbox.getRawButton(1))
  
    {
      pwr=pwr*0.1;
    }
    m_rightShooter.set(pwr);
    m_leftShooter.set(pwr);

  }


 
  public void stop() {
    m_rightShooter.stopMotor();;
    m_leftShooter.stopMotor();;
  }

 
}