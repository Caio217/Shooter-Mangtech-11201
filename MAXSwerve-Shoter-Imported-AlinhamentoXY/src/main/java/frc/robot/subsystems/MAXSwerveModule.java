// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d; 
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ModuleConstants;



import frc.robot.Configs;

/**
 * Representa um único módulo MAXSwerve.
 *
 * <p>Cada módulo tem um motor para tracionar a roda, um motor para girar a roda, um encoder
 * relativo para distância/velocidade e um encoder absoluto para saber o ângulo do módulo.
 *
 * <p>Do ponto de vista de programação orientada a objetos, esta classe transforma um conjunto de
 * hardware em um objeto Java com métodos simples: ler estado, ler posição, mandar novo setpoint e
 * zerar encoder.
 */
public class MAXSwerveModule {
  // Motor NEO/SPARK MAX responsável pela velocidade linear da roda.
  // "private final" protege o campo de acesso externo e impede trocar a referência depois do construtor.
  private final SparkMax m_drivingSpark;

  // Motor NEO/SPARK MAX responsável pelo ângulo (azimute) da roda.
  private final SparkMax m_turningSpark;

  // Encoder relativo do drive: mede deslocamento e velocidade da roda desde o reset.
  // Relativo significa que ele sabe quanto andou desde que foi zerado, não uma posição absoluta do campo.
  private final RelativeEncoder m_drivingEncoder;

  // Encoder absoluto do giro: mede o ângulo físico do módulo mesmo após reiniciar o robô.
  // Absoluto significa que a leitura representa uma posição real dentro de uma volta completa.
  private final AbsoluteEncoder m_turningEncoder;

  // Controladores PID embarcados nos SPARK MAX, usados para velocidade e posição.
  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  // Offset entre o zero do encoder absoluto e o zero desejado no chassi.
  // Cada módulo pode ser montado em uma orientação diferente; o offset corrige essa diferença.
  private double m_chassisAngularOffset = 0;

  // Último estado pedido ao módulo; mantido como referência interna do estado desejado.
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constrói um módulo MAXSwerve e configura motores, encoders e controladores PID.
   *
   * <p>Esta configuração assume o conjunto REV MAXSwerve com SPARK MAX, motor brushless e encoder
   * absoluto no giro do módulo.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    // Parâmetros do construtor são valores recebidos de fora. Aqui vêm do DriveSubsystem/Constants.
    // Cria os controladores físicos usando os IDs CAN definidos em Constants.
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    // Busca os sensores conectados aos SPARKs antes de aplicar setpoints.
    // getEncoder() e getAbsoluteEncd oder() não criam sensores físicos; apenas retornam interfaces de leitura.
    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();



    // Interfaces usadas para mandar referências de velocidade/posição aos controladores REV.
    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    // ResetMode.kResetSafeParameters limpa configurações perigosas antes de aplicar as novas.
    // PersistMode.kPersistParameters grava no controlador para manter após desligar o robô.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        // Correção manual de sentido para o módulo cujo drive está no CAN 5.
        // Mantida como está para preservar a calibração física atual do robô.
        // setInverted(true) troca o sinal da saída do motor: positivo passa a girar no sentido oposto.
    if (drivingCANId == 5) {
      m_drivingSpark.setInverted(true);

    }


    // Guarda o offset de montagem e inicializa o estado desejado com o ângulo real atual.
    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());

    // Zera apenas distância percorrida; o ângulo vem do encoder absoluto.
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Retorna o estado atual do módulo.
   *
   * @return velocidade da roda e ângulo atual, já corrigido para o referencial do chassi.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    // Retorna o estado medido no referencial do chassi para odometria e debug.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /** Velocidade medida da roda em m/s, usando o fator de conversão configurado em Configs. */
  public double getDriveVelocityMetersPerSecond() {
  // Como o conversion factor foi configurado, getVelocity() já volta em metros por segundo.
  return m_drivingEncoder.getVelocity();
}

/** Ângulo atual do módulo já descontando o offset de montagem no chassi. */
public Rotation2d getTurningRotation() {
  // Rotation2d é o tipo da WPILib para ângulos; evita trabalhar com double cru espalhado.
  return Rotation2d.fromRadians(
      m_turningEncoder.getPosition() - m_chassisAngularOffset
  );
}




  /** Estado medido do módulo, usado pelo PathPlanner para estimar velocidade real do robô. */
  public SwerveModuleState getMeasuredState() {
  // Reúne duas leituras separadas em um SwerveModuleState, que é o formato esperado pela cinemática.
  return new SwerveModuleState(
      getDriveVelocityMetersPerSecond(),
      getTurningRotation()
  );
}


  /**
   * Retorna a posição acumulada do módulo.
   *
   * @return distância percorrida pela roda e ângulo atual, usados pela odometria.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    // A posição combina metros percorridos pelo drive e ângulo atual do módulo.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Define o estado desejado do módulo.
   *
   * @param desiredState estado desejado com velocidade em m/s e ângulo da roda.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    // Converte o ângulo desejado do referencial do chassi para o referencial do encoder absoluto.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    // Se for mais curto inverter a roda e girar menos, a WPILib otimiza o estado automaticamente.
    // Exemplo: em vez de girar 170 graus, pode girar 10 graus e inverter a velocidade da roda.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    // Drive usa controle de velocidade; turning usa controle de posição angular.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    // Guarda o pedido original, sem offset, para manter a semântica de estado desejado do módulo.
    m_desiredState = desiredState;
  }

  /** Zera os encoders relativos do módulo que acumulam distância. */
  public void resetEncoders() {
    // Zera só o encoder relativo de distância; o absoluto de direção não precisa ser zerado.
    m_drivingEncoder.setPosition(0);
  }

  
}
