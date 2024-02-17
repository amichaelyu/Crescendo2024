package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final TalonFX  m_rightClimberMotor = new TalonFX(ClimberConstants.rightClimberMotorID);
  private final TalonFX m_leftClimberMotor = new TalonFX(ClimberConstants.leftClimberMotorID);
  private boolean isHomed;

  /** Creates a new Climber. */
  public Climber() {
    m_rightClimberMotor.getConfigurator().apply(ClimberConstants.talonFXConfigs);
    m_leftClimberMotor.getConfigurator().apply(ClimberConstants.talonFXConfigs);

    m_rightClimberMotor.setNeutralMode(NeutralModeValue.Brake);
    m_leftClimberMotor.setNeutralMode(NeutralModeValue.Brake);
    m_rightClimberMotor.setPosition(0);
    m_leftClimberMotor.setPosition(0);

    m_rightClimberMotor.setInverted(true);
    m_leftClimberMotor.setInverted(true);
    isHomed = false;
  }

  public void home() {
    m_rightClimberMotor.setControl(new VoltageOut(-1));
    m_leftClimberMotor.setControl(new VoltageOut(-1));
  }

  public void goSetpoint(double setPoint) {
    if (isHomed) {
      m_rightClimberMotor.setControl(new MotionMagicVoltage(setPoint));
    }
  }
  
  public void move(double pwr) {
    m_rightClimberMotor.set(pwr);
    m_leftClimberMotor.set(pwr);
  }


 
  public void stop() {
    m_rightClimberMotor.stopMotor();
  }

 
}