package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TilterConstants;

public class Tilter extends SubsystemBase {
  private final TalonFX m_tilterMotor = new TalonFX(TilterConstants.tilterMotorID);
  private boolean isHomed;
  private double setpoint = 0;
  private DigitalInput limitSwitchTop;
  private final DigitalInput limitSwitchBottom;

  /**
   * Creates a new Tilter.
   */
  public Tilter() {
    limitSwitchBottom = new DigitalInput(TilterConstants.kLIFTER_LIMIT_BOTTOM);

    m_tilterMotor.getConfigurator().apply(TilterConstants.talonFXConfigs);
    m_tilterMotor.setPosition(TilterConstants.IDLE_POSITION);

//    m_tilterMotor.setNeutralMode(NeutralModeValue.Brake);

//    m_tilterMotor.setInverted(true);
    isHomed = false;
//    SmartDashboard.putNumber("tilter voltage", 0);
    SmartDashboard.putNumber("tilter p", 0.05); // 0.05
    SmartDashboard.putNumber("tilter kS", 2);
    SmartDashboard.putNumber("tilter magicVel", 50);
    SmartDashboard.putNumber("tilter magicAcc", 10);
  }

  @Override
  public void periodic() {
//      setVoltage(SmartDashboard.getNumber("tilter voltage", 0));
    SmartDashboard.putNumber("tilter rotations", m_tilterMotor.getPosition().getValue());

    TalonFXConfiguration tempConfig = TilterConstants.talonFXConfigs;
    tempConfig.Slot0.kP = SmartDashboard.getNumber("tilter p", 0);
    tempConfig.MotionMagic.MotionMagicCruiseVelocity = SmartDashboard.getNumber("tilter magicVel", 50);
    tempConfig.MotionMagic.MotionMagicAcceleration = SmartDashboard.getNumber("tilter magicAcc", 10);
    m_tilterMotor.getConfigurator().apply(tempConfig);

    SmartDashboard.putBoolean("At Bottom", isAtBottom());
    if (isAtBottom()) {
      homed();
//      System.out.println("Lifter at Bottom; not going down.");
    }
  }

  public void setVoltage(double voltage) {
      if (isAtBottom() && voltage < 0) {
        m_tilterMotor.setControl(new VoltageOut(0));
        System.out.println("Lifter at Bottom; not going down.");
      }
      else{
        m_tilterMotor.setControl(new VoltageOut(voltage));
      }
    }

  public void resetEncoder() {
    m_tilterMotor.setPosition(0);
  }

  public void homed() {
    isHomed = true;
//    SoftwareLimitSwitchConfigs config = TilterConstants.talonFXConfigs.SoftwareLimitSwitch;
//    config.ReverseSoftLimitEnable = true;
//    config.ForwardSoftLimitEnable = true;
//    m_tilterMotor.getConfigurator().apply(config);
    resetEncoder();
  }

  public boolean atSetpoint() {
    return (getPosition() > (setpoint - TilterConstants.PID_TOLERANCE)) && (getPosition() < (setpoint + TilterConstants.PID_TOLERANCE));
  }

  public void setPosition(double position) {
    setpoint = position;
    if (isHomed) {
      if (m_tilterMotor.getPosition().getValue() > position) {
        m_tilterMotor.setControl(new MotionMagicVoltage(position).withFeedForward(-m_tilterMotor.getPosition().getValue() * 0.01));
//        Slot0Configs config = TilterConstants.talonFXConfigs.Slot0;
//        config.kS = SmartDashboard.getNumber("tilter kS up", 2);
//        config.kS = 1;
//        m_tilterMotor.getConfigurator().apply(config);
      }
      else {
        m_tilterMotor.setControl(new MotionMagicVoltage(position).withFeedForward(m_tilterMotor.getPosition().getValue() * 0.01));
//        Slot0Configs config = TilterConstants.talonFXConfigs.Slot0;
//        config.kS = 1;
////        SmartDashboard.putNumber("tilter kS", 1);
////        config.Slot0.kS = 2;
//        config.kS = SmartDashboard.getNumber("tilter kS up", 2);
//        m_tilterMotor.getConfigurator().apply(config);
      }
    }
  }

  public double getPosition() {
    return m_tilterMotor.getPosition().getValue();
  }

  public void move(double pwr) {
    if (isAtBottom() && pwr < 0) {
      m_tilterMotor.set(0);
      System.out.println("Lifter at Bottom; not going down.");
    }
    else{
      m_tilterMotor.set(pwr);
    }
  }

 
  public void stop() {
    m_tilterMotor.stopMotor();
  }

  public boolean isAtBottom() {
		return !limitSwitchBottom.get();
    }
  }