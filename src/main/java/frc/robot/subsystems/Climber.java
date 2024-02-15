package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climber extends SubsystemBase {
  private final TalonFX  m_rightClimberMotor = new TalonFX(indexMotorID);
  private final TalonFX m_leftClimberMotor = new TalonFX(indexMotorID);



  /** Creates a new Climber. */
  public Climber() {
    

    m_rightClimberMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_leftClimberMotor.getConfigurator().apply(new TalonFXConfiguration());


    m_rightClimberMotor.setInverted(true);
    m_leftClimberMotor.setInverted(true);

    m_rightClimberMotor.setNeutralMode(null);;
    m_leftClimberMotor.setNeutralMode(null);;
    
  }

  
  public void move(double pwr) {
   
    m_rightClimberMotor.set(pwr);
    m_leftClimberMotor.set(pwr);

  }


 
  public void stop() {
    m_rightClimberMotor.stopMotor();
    m_leftClimberMotor.stopMotor();
  }

 
}