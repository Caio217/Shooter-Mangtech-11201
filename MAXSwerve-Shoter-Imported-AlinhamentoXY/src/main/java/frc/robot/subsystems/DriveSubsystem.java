package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

/**
 * Subsistema responsável pelo drivetrain swerve do robô.
 *
 * <p>Ele reúne quatro módulos MAXSwerve, o giroscópio Pigeon2, odometria e os métodos que comandos
 * de teleop/autônomo usam para mover o robô.
 *
 * <p>Em command-based, um subsistema representa uma parte física do robô que deve ter dono único.
 * Comandos que exigem {@code DriveSubsystem} não devem rodar ao mesmo tempo que outros comandos que
 * também controlam o drive.
 */
public class DriveSubsystem extends SubsystemBase {

  // Giroscópio usado para heading do robô, odometria e controle field-relative.
  // O ID 0 é o CAN ID do Pigeon2. Se mudar no Phoenix Tuner, precisa mudar aqui também.
  private final Pigeon2 m_pigeon = new Pigeon2(0);

  // Módulo dianteiro esquerdo: recebe CAN IDs e offset angular definidos em Constants.
  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  // Módulo dianteiro direito.
  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  // Módulo traseiro esquerdo.
  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  // Módulo traseiro direito.
  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  // Odometria estima a pose do robô no campo combinando gyro + posições dos quatro módulos.
  // Pose2d = posição X/Y no campo + rotação do robô. Essa estimativa é essencial para autônomo.
  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_pigeon.getRotation2d(),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
          });

  public DriveSubsystem() {

    // Construtor do subsistema: roda uma vez quando RobotContainer cria o DriveSubsystem.
    // Aplica configuração padrão ao Pigeon2 e zera o yaw ao ligar o subsistema.
    Pigeon2Configuration config = new Pigeon2Configuration();
    m_pigeon.getConfigurator().apply(config);
    m_pigeon.setYaw(0);

    // Informa à HAL/Driver Station que o robô usa um swerve MAXSwerve.
    HAL.report(
        tResourceType.kResourceType_RobotDrive,
        tInstances.kRobotDriveSwerve_MaxSwerve
    );
  }

  @Override
  public void periodic() {
    // Atualiza a pose estimada a cada ciclo usando as leituras atuais dos encoders e do gyro.
    // periodic() é chamado pelo CommandScheduler porque Robot.robotPeriodic() roda o scheduler.
    m_odometry.update(
        m_pigeon.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Publica o heading para facilitar debug no SmartDashboard durante testes.
    SmartDashboard.putNumber("Gyro Heading", getHeading());
  }

  /** Retorna a pose estimada do robô no campo, usada pelo PathPlanner e por comandos autônomos. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /** Define uma pose conhecida para a odometria, normalmente no início de um autônomo. */
  public void resetOdometry(Pose2d pose) {
    // resetPosition precisa receber: rotação atual do gyro, posições atuais dos módulos e pose desejada.
    // Isso evita salto incorreto quando uma trajetória começa em uma posição conhecida.
    m_odometry.resetPosition(
        m_pigeon.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Move o robô a partir de velocidades normalizadas.
   *
   * @param xSpeed velocidade para frente/trás de -1 a 1
   * @param ySpeed velocidade lateral de -1 a 1
   * @param rot rotação de -1 a 1
   * @param fieldRelative se true, interpreta x/y no referencial do campo; se false, no do robô
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    // Converte comandos normalizados para unidades reais antes da cinemática swerve.
    // ChassisSpeeds é a forma padrão da WPILib para dizer "quero mover o chassi assim".
    ChassisSpeeds speeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                // Field-relative: frente do joystick significa frente do campo, compensando o gyro.
                xSpeed * DriveConstants.kMaxSpeedMetersPerSecond,
                ySpeed * DriveConstants.kMaxSpeedMetersPerSecond,
                rot * DriveConstants.kMaxAngularSpeed,
                m_pigeon.getRotation2d())
            : new ChassisSpeeds(
                // Robot-relative: x/y são interpretados em relação à frente atual do robô.
                xSpeed * DriveConstants.kMaxSpeedMetersPerSecond,
                ySpeed * DriveConstants.kMaxSpeedMetersPerSecond,
                rot * DriveConstants.kMaxAngularSpeed);

    driveRobotRelative(speeds);
  }

  /** Comanda o ângulo fixo usado pelo projeto como posição de trava/orientação do swerve. */
  public void setX() {
    // Cada módulo recebe velocidade zero e ângulo de 45 graus.
    // O método é chamado enquanto RB do piloto está pressionado.
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /** Zera o yaw do Pigeon2; usado pelo botão START do piloto para redefinir a frente do robô. */
  public void zeroHeading() {
    // Zerar yaw define a orientação atual como "frente" para teleop e field-relative.
    m_pigeon.setYaw(0);
  }

  /** Retorna o heading atual do robô em graus. */
  public double getHeading() {
    // getRotation2d vem do Pigeon2 e é convertido para graus para leitura humana no dashboard.
    return m_pigeon.getRotation2d().getDegrees();
  }

  /* ===== PATHPLANNER ===== */

  /** Mede a velocidade atual do chassi a partir dos estados reais dos módulos. */
  public ChassisSpeeds getChassisSpeeds() {
    // toChassisSpeeds faz o caminho inverso da cinemática: estados dos módulos -> velocidade do chassi.
    // O PathPlanner usa isso para comparar velocidade desejada e velocidade medida.
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getMeasuredState(),
        m_frontRight.getMeasuredState(),
        m_rearLeft.getMeasuredState(),
        m_rearRight.getMeasuredState()
    );
  }


  /** Recebe velocidades no referencial do robô e distribui setpoints para os quatro módulos. */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    // Cinemática converte velocidade desejada do chassi em velocidade/ângulo de cada roda.
    // A ordem do array gerado segue a ordem definida em Constants.kDriveKinematics.
    SwerveModuleState[] states =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    // Reduz proporcionalmente velocidades se alguma roda passaria do limite físico configurado.
    // Isso mantém o movimento proporcional em vez de simplesmente cortar uma roda isolada.
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states,
        DriveConstants.kMaxSpeedMetersPerSecond);


    // Envia setpoints calculados para os módulos dianteiros.
    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);

    // m_rearLeft.setDesiredState(states[2]);
    // m_rearRight.setDesiredState(states[3]);

      // Correção manual observada no projeto: inverte a velocidade do traseiro esquerdo
      // antes de enviar o estado. Mantida sem alteração para preservar o comportamento atual.
      // Este tipo de correção geralmente vem de montagem mecânica, inversão de motor ou orientação do módulo.
      SwerveModuleState rearLeftFixed  = new SwerveModuleState(-states[2].speedMetersPerSecond, states[2].angle);
      SwerveModuleState rearRightFixed = new SwerveModuleState(states[3].speedMetersPerSecond, states[3].angle);
      m_rearLeft.setDesiredState(rearLeftFixed);
      m_rearRight.setDesiredState(rearRightFixed);
  }
}
