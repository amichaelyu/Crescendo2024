package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {
  private final TalonFX m_rightShooter = new TalonFX(ShooterConstants.rightShooterID);
  private final TalonFX m_leftShooter = new TalonFX(ShooterConstants.leftShooterID);

  public Shooter() {
    m_rightShooter.getConfigurator().apply(ShooterConstants.talonFXConfigs);
    m_rightShooter.setInverted(true);
    m_rightShooter.setNeutralMode(NeutralModeValue.Coast);
    m_leftShooter.setNeutralMode(NeutralModeValue.Coast);

    m_leftShooter.setControl(new Follower(ShooterConstants.rightShooterID, true));

    SmartDashboard.putNumber("shooter p", 0);
//    SmartDashboard.putNumber("shooter voltage", 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter actual speed", m_rightShooter.getVelocity().getValue());
    TalonFXConfiguration tempConfig = ShooterConstants.talonFXConfigs;
    tempConfig.Slot0.kP = SmartDashboard.getNumber("shooter p", 0);
    m_rightShooter.getConfigurator().apply(tempConfig);

//    setVoltage(SmartDashboard.getNumber("shooter voltage", 0));
  }

  public void setSpeed(double speed) {
    m_rightShooter.setControl(new MotionMagicVelocityVoltage(speed).withFeedForward(ShooterConstants.FEEDFORWARD));
  }

  public void setVoltage(double volts) {
    m_rightShooter.setControl(new VoltageOut(volts));
  }
 
  public void stop() {
    m_rightShooter.stopMotor();
    m_leftShooter.stopMotor();
  }
}