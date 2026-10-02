package frc.robot;

// Configurações REV são objetos que descrevem como o SPARK MAX deve se comportar.
// Elas são aplicadas nos motores dentro de MAXSwerveModule, não executam o motor sozinhas.
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

/**
 * Agrupa configurações de hardware aplicadas aos controladores REV.
 *
 * <p>Separar essas configurações do subsistema evita duplicar parâmetros de encoder, PID e limites
 * de corrente em cada módulo swerve.
 *
 * <p>Do ponto de vista de Java, esta classe é um "molde de configuração": o bloco {@code static}
 * roda uma vez quando a classe é carregada e preenche os objetos {@code drivingConfig} e
 * {@code turningConfig}. Depois, cada módulo aplica esses mesmos objetos aos seus SPARK MAX.
 */
public final class Configs {

  // Configuração compartilhada pelos quatro módulos MAXSwerve.
  // Classe interna separa o que é do swerve de possíveis configurações futuras de outros mecanismos.
  public static final class MAXSwerveModule {

    // Config do motor que empurra a roda para frente/trás.
    // "new SparkMaxConfig()" cria um objeto vazio que será preenchido no bloco static abaixo.
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();

    // Config do motor que gira o azimute da roda.
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Bloco static: executa uma vez para configurar valores globais da classe.
      // Isso é diferente de um construtor, que executaria a cada novo objeto criado.

      // ===== FATORES DE CONVERSAO =====
      // Converte voltas do encoder relativo do motor de drive em metros percorridos pela roda.
      // Circunferência da roda = diâmetro * PI; divide pela redução para sair de voltas do motor.
      double drivingPositionFactor =
          ModuleConstants.kWheelDiameterMeters * Math.PI
              / ModuleConstants.kDrivingMotorReduction;

      // O encoder absoluto de direção representa uma volta completa do módulo em radianos.
      // 2 * PI radianos = 360 graus.
      double turningPositionFactor = 2 * Math.PI;

      // Feedforward simples para o controle de velocidade do drive.
      // Quanto maior a velocidade livre configurada, menor fica o ganho base aplicado.
      double drivingVelocityFF =
          1.0 / ModuleConstants.kDriveWheelFreeSpeedRps;

      // ===== CONFIGURACAO DO MOTOR DE DRIVE =====
      // IdleMode.kBrake faz o motor resistir movimento quando o comando vai para zero.
      // smartCurrentLimit protege motor, controlador e bateria contra corrente alta demais.
      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);

      // A WPILib espera posição em metros e velocidade em m/s para odometria e cinemática.
      // Sem esses fatores, a odometria receberia rotações de motor em vez de distância física.
      drivingConfig.encoder
          .positionConversionFactor(drivingPositionFactor) // metros
          .velocityConversionFactor(drivingPositionFactor / 60.0); // m/s

      // PID embarcado no SPARK MAX mantém a velocidade desejada de cada roda.
      // FeedbackSensor.kPrimaryEncoder usa o encoder relativo interno do motor de drive.
      // pid(P, I, D): P reage ao erro atual, I acumula erro, D reage à variação do erro.
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(0.04, 0.0, 0.0)
          .velocityFF(drivingVelocityFF)
          .outputRange(-1.0, 1.0);

      // ===== CONFIGURACAO DO MOTOR DE TURN =====
      // O motor de turn não precisa girar rápido, mas precisa segurar ângulo com estabilidade.
      turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20);

      // Encoder absoluto permite saber o ângulo real do módulo mesmo após reiniciar o robô.
      // inverted(true) corrige o sentido de leitura do encoder conforme a montagem física.
      turningConfig.absoluteEncoder
          .inverted(true)
          .positionConversionFactor(turningPositionFactor) // rad
          .velocityConversionFactor(turningPositionFactor / 60.0); // rad/s

      // PID de posição do azimute usa wrapping para escolher o menor caminho entre 0 e 2pi.
      // Position wrapping faz 0 rad e 2pi rad serem tratados como a mesma direção física.
      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(1.0, 0.0, 0.0)
          .outputRange(-0.8, 0.8)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0.0, turningPositionFactor);
    }
  }
}
