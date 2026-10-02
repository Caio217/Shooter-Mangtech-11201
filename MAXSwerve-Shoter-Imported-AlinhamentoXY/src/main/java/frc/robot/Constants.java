package frc.robot;

// Imports trazem classes prontas de bibliotecas externas para este arquivo.
// Aqui usamos tipos matemáticos da WPILib, que já sabem trabalhar com unidades e cinemática FRC.
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * Centraliza valores fixos usados pelo robô.
 *
 * <p>Em projetos FRC, manter portas CAN, dimensões físicas e limites de controle aqui facilita
 * ajustes durante testes sem precisar procurar números espalhados pelo código.
 *
 * <p>Esta classe usa o padrão "constants holder": não representa uma peça física sozinha, apenas
 * organiza números que outros arquivos consultam. Por isso ela é {@code final} e seus valores são
 * {@code public static final}: públicos para serem acessados de outros arquivos, estáticos para não
 * precisar criar objeto, e finais para não mudar durante a execução.
 */
public final class Constants {

  /* ===================== DRIVE ===================== */
  // Classe interna usada apenas para agrupar constantes relacionadas ao drivetrain.
  // Em Java, "Constants.DriveConstants.kMaxSpeedMetersPerSecond" deixa claro de onde vem o valor.
  public static final class DriveConstants {

    // Velocidade linear máxima teórica usada para converter comandos normalizados (-1 a 1) em m/s.
    // Se o piloto manda 1.0 para frente, o DriveSubsystem multiplica por este valor.
    public static final double kMaxSpeedMetersPerSecond = 4.8;

    // Velocidade angular máxima usada para converter rotação normalizada em rad/s.
    // 2 * PI rad/s equivale a aproximadamente uma volta completa por segundo.
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    // Distância lateral entre rodas esquerda/direita e distância frontal/traseira do chassi.
    // Units.inchesToMeters evita misturar polegadas físicas do robô com metros usados pela WPILib.
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    public static final double kWheelBase  = Units.inchesToMeters(26.5);

    // Geometria do swerve: posição de cada módulo em relação ao centro do robô.
    // A ordem precisa bater com a ordem usada no DriveSubsystem.
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            // Cada Translation2d é uma coordenada (x, y) do módulo em metros.
            // x positivo fica para frente; y positivo fica para a esquerda do robô.
            new Translation2d( kWheelBase / 2,  kTrackWidth / 2),
            new Translation2d( kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2,  kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    /* -------- CAN IDs -------- */
    // IDs CAN dos motores de drive e giro de cada módulo swerve.
    // Estes valores devem ser iguais aos configurados nos SPARK MAX pelo REV Hardware Client.
    // Cada par "Driving/Turning" representa um módulo: um motor anda, outro aponta a roda.
    public static final int kFrontLeftDrivingCanId  = 3;
    public static final int kFrontLeftTurningCanId  = 4;

    public static final int kFrontRightDrivingCanId = 7;
    public static final int kFrontRightTurningCanId = 8;

    public static final int kRearLeftDrivingCanId   = 1;
    public static final int kRearLeftTurningCanId   = 2;

    public static final int kRearRightDrivingCanId  = 5;
    public static final int kRearRightTurningCanId  = 6;

    // IDs reservados para mecanismos de manipulação de peça; o RobotContainer instancia os motores.
    // Observação técnica: estes nomes existem aqui, mas o código atual instancia shooter/intake direto
    // no RobotContainer. Se a equipe centralizar depois, conferir se os IDs batem com a fiação real.
    public static final int shoter  = 10;
    public static final int inTaker  = 11;



    /* -------- OFFSETS DOS MODULOS -------- */
    // Correções angulares para alinhar o zero do encoder absoluto com a frente real do robô.
    // Se um módulo aponta para o lado ao comandar "frente", o ajuste normalmente fica aqui.
    // O valor está em radianos, porque as classes de cinemática da WPILib usam radianos internamente.
    public static final double kFrontLeftChassisAngularOffset  = 0 - Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset   = 0;
    public static final double kBackRightChassisAngularOffset  = 0 - Math.PI / 2;

    // Mantido para indicar se o giroscópio deveria ser invertido; no código atual o Pigeon é usado direto.
    public static final boolean kGyroReversed = false;
  }

  /* ===================== MODULE ===================== */
  // Constantes físicas do módulo de roda, usadas para converter rotações de motor em metros.
  public static final class ModuleConstants {

    // MAXSwerve padrão (3")
    // 0.0762 m = 3 polegadas. Alterar se a roda física for trocada por outro diâmetro.
    public static final double kWheelDiameterMeters = 0.0762;

    // Redução típica do MAXSwerve
    // Relação entre voltas do motor e voltas da roda; entra no cálculo de distância percorrida.
    public static final double kDrivingMotorReduction = 6.75;

    // Velocidade livre aproximada da roda
    // Usada no feedforward do SPARK MAX para estimar quanta saída aplicar para uma velocidade.
    public static final double kDriveWheelFreeSpeedRps = 4.5;
  }

  /* ===================== OI ===================== */
  // OI significa Operator Interface: controles, botões e zonas mortas usados pelos operadores.
  public static final class OIConstants {
    // Porta USB do controle principal de pilotagem na Driver Station.
    public static final int kDriverControllerPort = 0;

    // Porta USB do controle dedicado ao shooter/intake/elevador.
    public static final int kShoterControllerPort = 1;

    // Ignora pequenas leituras dos analógicos para evitar drift quando o joystick está solto.
    public static final double kDriveDeadband = 0.05;
  }

  /* ===================== AUTO (GUARDADO) ===================== */
  // Constantes de autônomo que podem ser usadas em trajetórias ou controladores futuros.
  public static final class AutoConstants {
    // Restrição de perfil trapezoidal para rotação autônoma, mantida para rotinas futuras.
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(Math.PI, Math.PI);
  }
}
