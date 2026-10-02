package frc.robot;


import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.VisionAlignStation;







/**
 * Liga todos os subsistemas, comandos, controles e opções autônomas do robô.
 *
 * <p>No padrão command-based da WPILib, esta classe é o "mapa elétrico/lógico" do código: aqui
 * ficam os motores dos mecanismos, o drive, os botões do controle e as rotinas que o PathPlanner
 * pode chamar durante o autônomo.
 *
 * <p>Para estudar Java: os campos declarados no começo da classe são atributos do objeto
 * RobotContainer. O construtor {@code RobotContainer()} inicializa esses atributos e configura
 * comandos. Os métodos privados organizam tarefas internas, e os métodos públicos são chamados por
 * outras classes, como Robot.java.
 */
public  class RobotContainer {

    // Timer usado para controlar ciclos automáticos do intake (ex: puxar bola e disparar)
    //private final Timer intakeToggleTimer = new Timer();

    // Indica se o modo LT (modo automático de disparo) está ativo
    //private boolean ltModeActive = false;

    // Guarda o momento em que o gatilho LT foi pressionado
    // usado para calcular o tempo de funcionamento do shooter
    // Variável de instância: mantém valor entre várias chamadas do comando enquanto o robô está ligado.
    private double ltStartTime = 0.0;

    /* ================= INTAKE TIMING ================= */

    // Tempo que o intake fica puxando a bola antes do disparo automático
    //private static final double INTAKE_PULL_TIME = 1.1;

    // Tempo que o intake fica empurrando a bola para o shooter
    //private static final double INTAKE_SHOOT_TIME = 0.65;

    // Permite escolher qual rotina autônoma rodará antes da partida
    // aparece no SmartDashboard como menu de seleção
    // SendableChooser<Command> é genérico: o tipo entre < > diz que ele guarda objetos Command.
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();



  /* ================= SUBSYSTEM ================= */

    // Subsistema responsável por controlar o swerve drive do robô
    // controla locomoção, odometria e movimento autônomo
    // Apenas uma instância do DriveSubsystem é criada para evitar dois donos do mesmo hardware.
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    /* ================= SHOOTER / INTAKE ================= */
    // Motor do shooter: acelera a roda/roletes que lançam a peça de jogo.
    // SparkMax(11, kBrushless) cria a interface Java para o controlador CAN 11 com motor brushless.
    private final SparkMax shooterMotor = new SparkMax(11, MotorType.kBrushless);

    // Motor do intake: puxa a peça para dentro do robô ou empurra para o shooter.
    // O sentido positivo/negativo depende da montagem física; por isso os comandos usam sinais testados.
    private final SparkMax intakeMotor  = new SparkMax(10, MotorType.kBrushless);
    //
    /* ================= ELEVADOR ================= */
    // Motor que movimenta o elevador vertical do robô
    // Diferente do drive, este mecanismo está direto no RobotContainer e não em um subsistema separado.
    private final SparkMax elevatorMotor = new SparkMax(12, MotorType.kBrushless);

    // Encoder que mede a posição atual do elevador
    // getEncoder() retorna o sensor relativo integrado/associado ao SPARK MAX do elevador.
    private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

    // Posição mínima permitida do elevador (posição inicial)
    // Estes limites são em unidades do encoder relativo, não necessariamente metros ou centímetros.
    private static final double ELEVATOR_MIN = 200;

    // Posição máxima permitida do elevador (altura máxima)
    private static final double ELEVATOR_MAX = -100.0;

    // Zona segura do elevador (posição intermediária usada em automações)
    // Calculada a partir de ELEVATOR_MIN para deixar uma margem antes do limite inferior.
    private static final double ELEVATOR_SAFE_ZONE = ELEVATOR_MIN - 20;

    /* ================= APRILTAG ================= */
    // Guarda o ID da última AprilTag detectada
    //private int lastTagID = -1;

    // Distância atual entre a câmera do robô e a AprilTag detectada
    // Atualizada no comando padrão do drive; usada quando o modo de visão está ligado.
    private double tagDistance = 0.0;

    // Indica se a câmera está vendo uma AprilTag válida
    // boolean guarda apenas true/false, ideal para decisões com if.
    private boolean hasValidTag = false;

    // Deslocamento lateral entre o robô e a tag
    // usado para centralizar o robô lateralmente
    //private double tagY = 0.0;

    // Ângulo entre o robô e a tag (Yaw)
    // usado para alinhar o robô corretamente
    // Yaw vem em graus no PhotonVision; depois é tratado com MathUtil.inputModulus.
    private double tagAngle = 0.0;

    /* ================= VIBRAÇÃO ================= */
    // Indica se o modo vibração (confirmação de alinhamento) está ativo
    //private boolean vibrationModeActive = false;
    // ESSE É UM EXEMPLO APENAS
    // Guarda o momento em que a vibração começou
    private double vibTime = 0.0;

    // Indica se a distância do robô em relação à tag já foi travada
   // private boolean distanceLocked = false;

    // Máquina de estados do sistema de alinhamento por visão
    // enum cria um conjunto fechado de estados possíveis, evitando strings soltas como "ligado".
    private enum VisionState {
        OFF,       // sistema desligado
        APPROACH,  // robô se aproximando da tag
        VIBRATE    // robô vibra indicando alinhamento completo
    }

    // Estado atual do sistema de visão
    private VisionState visionState = VisionState.OFF;
   
    // Contador usado para garantir que o robô permaneça alinhado
    // por alguns ciclos antes de considerar alinhamento final
    private int inRangeCounter = 0;

    // Distância filtrada da tag (usando filtro EMA para reduzir ruído da câmera)
    // Filtros ajudam porque visão computacional pode variar alguns centímetros entre frames.
    private double filteredDistance = 0.0;

    // Indica se o filtro de distância já foi inicializado
    private boolean filterInit = false;

    // Distância alvo entre robô e AprilTag
    // Constante de calibração do alinhamento: mudar este valor muda onde o robô tenta parar.
    private static final double TARGET_DIST = 1.6;

    // Quantidade de ciclos consecutivos necessários para confirmar alinhamento
    private static final int STABLE_LOOPS = 6;

    // Ganho proporcional para correção de distância
    // Controle P: velocidade = erro * ganho, depois limitada por MAX_X_SPEED.
    private static final double KP_DIST = 0.55;

    // Limite máximo de velocidade durante aproximação
    //private static final double MAX_APPROACH_SPEED = 0.2;

    // Intensidade da vibração lateral do robô
    private static final double VIB_SPEED = 0.25;

    // Frequência da vibração (oscilações por segundo)
    // Usada dentro de Math.sin para alternar rapidamente o comando lateral.
    private static final double VIB_FREQ  = 35.0;

    // Tolerância aceitável para distância da AprilTag
    private static final double DIST_TOL = 0.02;

    // Tolerância lateral aceitável
    //private static final double Y_TOL = 0.005;

    // Tolerância angular aceitável
    private static final double ANGLE_TOL = 3.0;

    // Ganho proporcional para correção lateral
    //private static final double KP_Y = 1.2;

    // Ganho proporcional para correção de rotação
    // Como tagAngle está em graus, este ganho converte graus de erro em comando normalizado de rotação.
    private static final double KP_ROT = 0.015;

    // Velocidade máxima para correção de distância
    private static final double MAX_X_SPEED = 0.3;

    // Velocidade máxima para correção lateral
    //private static final double MAX_Y_SPEED = 0.15;

    // Velocidade máxima de rotação
    private static final double MAX_ROT_SPEED = 0.3;

    /* ================= SHOOTER DECAY CONFIG ================= */

    // Tempo entre cada redução de velocidade (segundos)
    // Usado para calcular degraus de velocidade durante o disparo com LT.
    private static final double SHOOTER_STEP_TIME = 1;

    // Velocidade inicial do shooter
    private static final double SHOOTER_START_SPEED = 1.0;

    // Quanto a velocidade diminui a cada passo
    private static final double SHOOTER_STEP_DECREASE = 0.1;

    // Velocidade mínima permitida
    private static final double SHOOTER_MIN_SPEED = 0.7;




    /**
     * Configura o SPARK MAX do shooter.
     *
     * <p>O modo coast deixa o mecanismo desacelerar livremente, e a ausência de rampa faz o shooter
     * responder imediatamente ao gatilho/comando autônomo.
     */
    private void configureShooterMotor() {

    // Objeto temporário de configuração: existe só dentro deste método.
    SparkMaxConfig shooterConfig = new SparkMaxConfig();

        shooterConfig
            .idleMode(SparkMaxConfig.IdleMode.kCoast)   // shooter livre
            .smartCurrentLimit(40)                      // limite seguro p/ NEO
            .openLoopRampRate(0.0);                     // SEM RAMPA (instantâneo)

        shooterMotor.configure(
            // configure envia os parâmetros para o controlador físico SPARK MAX.
            shooterConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    /**
     * Configura o SPARK MAX do elevador.
     *
     * <p>O modo brake ajuda a segurar a posição do mecanismo quando o operador solta o botão.
     */
    private void configureElevatorMotor() {

    // Configuração separada porque o elevador precisa segurar posição, diferente do shooter.
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    elevatorConfig
        .idleMode(SparkMaxConfig.IdleMode.kBrake) // elevador segura quando solta
        .smartCurrentLimit(30);                  // limite seguro

    elevatorMotor.configure(
        elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );
}




  /* ================= CONTROLE ================= */
    // Controle 0: piloto do drive swerve.
    private final XboxController m_driverController =
      new XboxController(OIConstants.kDriverControllerPort);

    // Controle 1: operador dos mecanismos (shooter, intake, elevador e visão).
    private final XboxController m_ShoterController =
      new XboxController(OIConstants.kShoterControllerPort);

  /* ================= SUAVIDADE ================= */
  // Limitadores de aceleração suavizam joystick para evitar trancos no swerve.
  // SlewRateLimiter limita quanto o valor pode mudar por segundo; é como uma rampa em software.
  private final SlewRateLimiter m_xLimiter   = new SlewRateLimiter(1.2);
  private final SlewRateLimiter m_yLimiter   = new SlewRateLimiter(1.2);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1.8);

  /* ================= MODOS ================= */
  // Multiplicadores de velocidade usados pelo piloto para alternar entre precisão e velocidade.
  private enum DriveMode {
    FAST,
    NORMAL,
    SUPER_SLOW
  }

  /* ================= PHOTONVISION ================= */
    // Câmera frontal usada para AprilTags; o nome deve bater exatamente com o PhotonVision.
    private final PhotonCamera frontCamera = new PhotonCamera("Orbbec_Gemini_E_RGB_Camera"); // ATENCAO: nome EXATO da câmera no PhotonVision

    // Timer dedicado a rotinas de visão; iniciado no construtor.
    private final Timer visionTimer = new Timer();


  // Modo de drive inicial do piloto.
  private DriveMode driveMode = DriveMode.NORMAL;

  /* ================= CONSTRUTOR ================= */
  public RobotContainer() {    

    // Ordem do construtor importa: primeiro configura hardware, depois registra autos e bindings.
    // Inicia o timer usado por rotinas de visão/tempo.
    visionTimer.start();
    
    // Aplica configurações de segurança e comportamento aos motores de mecanismos.
    configureShooterMotor();

    configureElevatorMotor();
    // IMPORTANTE: o robô deve ligar com o elevador totalmente embaixo
    // setPosition(0) define o zero lógico do encoder relativo no momento em que o código inicia.
    elevatorEncoder.setPosition(0);

    // ================= REGISTER AUTO COMMANDS =================
    // Comandos nomeados permitem que trajetórias PathPlanner chamem ações do robô por nome.
    NamedCommands.registerCommand(
        "ShootAuto",
        // RunCommand executa a lambda repetidamente enquanto o comando estiver agendado.
        // withTimeout(5.0) encerra automaticamente depois de 5 segundos.
        new RunCommand(() -> {
            shooterMotor.set(0.8);
            intakeMotor.set(-1);
        }).withTimeout(5.0)
    );

    NamedCommands.registerCommand(
        "VisionAlignStation",
        // Rotina autônoma de alinhamento por AprilTag antes de uma ação de estação/alvo.
        new VisionAlignStation(m_robotDrive, frontCamera)
    );

    NamedCommands.registerCommand(
        "VisionReturnToBall",
        // Rotina autônoma de retorno/alinhamento usando outro conjunto de AprilTags.
        new VisionReturnToBall(m_robotDrive, frontCamera)
    );

    NamedCommands.registerCommand(
        "FullShootAuto",
        // Sequência completa de preparação, disparo e desligamento usada por trajetórias autônomas.
        // SequentialCommandGroup roda um comando por vez, na ordem em que aparecem abaixo.
        new SequentialCommandGroup(

            // FASE 1 — CARREGAR (1.5 segundos)
            new RunCommand(() -> {
                // Lambda sem parâmetros: código executado pelo scheduler enquanto a fase está ativa.
                shooterMotor.set(1);   // pré-rotação
                intakeMotor.set(1);    // puxando bola
            }).withTimeout(1.5),

            // FASE 2 — DISPARAR (1.5 segundos)
            new RunCommand(() -> {
                shooterMotor.set(0.65);
                intakeMotor.set(-1.0);
            }).withTimeout(0.5),

            // FASE 3 — Ultimas bolas (2 segundos)
            new RunCommand(() -> {
                shooterMotor.set(0.95);
                intakeMotor.set(-1.0);
            }).withTimeout(0.75),

            // FASE 4 — 
            new RunCommand(() -> {
                shooterMotor.set(0.85);
                intakeMotor.set(-1.0);
            }).withTimeout(2.0),

            // FASE 4 — DESLIGAR
            // InstantCommand executa uma ação uma única vez; aqui é usado para parar motores.
            new InstantCommand(() -> {
                shooterMotor.stopMotor();
                intakeMotor.stopMotor();
            })
        )
    );

    NamedCommands.registerCommand(
        "FullShootReturn",
        // Variante de disparo com tempos/velocidades diferentes para outra posição do autônomo.
        new SequentialCommandGroup(

            // FASE 1 — CARREGAR (1.5 segundos)
            new RunCommand(() -> {
                shooterMotor.set(1);   // pré-rotação
                intakeMotor.set(1);    // puxando bola
            }).withTimeout(1.5),

            // FASE 2 — DISPARAR (1.5 segundos)
            new RunCommand(() -> {
                shooterMotor.set(0.7);
                intakeMotor.set(-1.0);
            }).withTimeout(0.5),

            // FASE 3 — Ultimas bolas (2 segundos)
            new RunCommand(() -> {
                shooterMotor.set(0.9);
                intakeMotor.set(-1.0);
            }).withTimeout(0.75),

            // FASE 4 — 
            new RunCommand(() -> {
                shooterMotor.set(0.85);
                intakeMotor.set(-1.0);
            }).withTimeout(3.0),

            // FASE 4 — DESLIGAR
            new InstantCommand(() -> {
                shooterMotor.stopMotor();
                intakeMotor.stopMotor();
            })
        )
    );

    NamedCommands.registerCommand(
        "ElevatorUpAuto",
        // Sobe o elevador até o limite máximo medido pelo encoder relativo.
        // until(...) encerra o RunCommand quando a condição booleana passa a ser verdadeira.
        new SequentialCommandGroup(

            // FASE 1 – Subir até atingir limite
            new RunCommand(() -> {
                // if protege o mecanismo: só aplica potência se o encoder ainda não passou do limite.
                if (elevatorEncoder.getPosition() > ELEVATOR_MAX ) {
                    elevatorMotor.set(-0.4);
                } else {
                    elevatorMotor.stopMotor();
                }
            }).until(() -> elevatorEncoder.getPosition() <= ELEVATOR_MAX),

            // FASE 2 – Garantir parada
            new InstantCommand(() -> elevatorMotor.stopMotor())
        )
    );

    NamedCommands.registerCommand(
        "ElevatorDownAuto",
        // Desce o elevador até a região inferior segura antes de parar o motor.
        new SequentialCommandGroup(

            new RunCommand(() -> {
                if (elevatorEncoder.getPosition() < ELEVATOR_MIN - 20) {
                    elevatorMotor.set(0.4);
                } else {
                    elevatorMotor.stopMotor();
                }
            }).until(() -> elevatorEncoder.getPosition() >= ELEVATOR_MIN),

            new InstantCommand(() -> elevatorMotor.stopMotor())
        )
    );


    RobotConfig config;

        try {
            // Carrega massa, dimensões e módulos do arquivo PathPlanner em src/main/deploy.
            // try/catch transforma falha de leitura em erro claro durante a inicialização do robô.
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException("Erro ao carregar settings.json do PathPlanner", e);
        }

        // Conecta o DriveSubsystem ao AutoBuilder para executar trajetórias holonômicas.
        // AutoBuilder precisa saber como ler pose, resetar odometria, medir velocidade e comandar drive.
        AutoBuilder.configure(
            m_robotDrive::getPose,                 // Pose atual do robô (odometria)
            m_robotDrive::resetOdometry,            // Reset da odometria
            m_robotDrive::getChassisSpeeds,         // Velocidade atual do robô (ROBOT-relative)
            m_robotDrive::driveRobotRelative,       // Comando para mover o robô
            new com.pathplanner.lib.controllers.PPHolonomicDriveController(
                // PIDConstants do PathPlanner controlam erro de posição X/Y e erro de rotação.
                new com.pathplanner.lib.config.PIDConstants(3.0, 0.0, 0.0), // X/Y
                new com.pathplanner.lib.config.PIDConstants(3.0, 0.0, 0.0)  // Rotação
            ),
            config,
            () -> false,                            // Não espelhar lado do campo
            m_robotDrive
        );

        // ================= AUTO CHOOSER (AGORA SIM) =================
        // Opção padrão exibida no SmartDashboard antes da partida.
        // PathPlannerAuto carrega uma rotina pelo nome salvo no PathPlanner.
        autoChooser.setDefaultOption(
            "AutoBall_BLUE",
            new PathPlannerAuto("AutoBall_BLUE")
        );

        // Segunda opção de rotina autônoma, também criada no PathPlanner.
        autoChooser.addOption(
            "Double Shoot",
            new PathPlannerAuto("Auto_DoubleShoot")
        );

        // Publica o chooser para a Driver Station/SmartDashboard selecionar o autônomo.
        SmartDashboard.putData("Auto Mode", autoChooser);



    // Depois que comandos/autos existem, conecta botões a ações.
    configureButtonBindings();




    /* ========== COMANDO PADRÃO (TELEOPERADO) ========= */
    // Comando padrão é o comando que roda no DriveSubsystem quando nenhum outro comando exige o drive.
    // Aqui ele mistura pilotagem manual com um modo opcional de alinhamento por visão.
    m_robotDrive.setDefaultCommand(

        new RunCommand(() -> {

            // ================= ATUALIZA VISÃO =================
            // Mesmo no teleop normal, a câmera é lida para manter as variáveis de tag atualizadas.
            PhotonPipelineResult result = frontCamera.getLatestResult();

            if (result.hasTargets()) {
                // getBestTarget() escolhe a tag considerada mais confiável pelo PhotonVision.
                PhotonTrackedTarget target = result.getBestTarget();
                Transform3d camToTarget = target.getBestCameraToTarget();

                // Distância total câmera->tag e yaw da tag alimentam o modo de alinhamento.
                tagDistance = camToTarget.getTranslation().getNorm();
                tagAngle = target.getYaw();
                hasValidTag = true;  // rotação da tag
                 } else {
                // Sem alvos no frame atual, qualquer modo automático de visão deve parar por segurança.
                hasValidTag = false;
            }

            // ================= CALCULA DRIVE NORMAL =================
            double multiplier;

            // O modo escolhido pelos botões A/B/X altera a velocidade máxima percebida pelo piloto.
            // switch é uma forma organizada de escolher comportamento com base em um enum.
            switch (driveMode) {
                case FAST:
                    multiplier = 1.0;
                    break;
                case SUPER_SLOW:
                    multiplier = 0.2;
                    break;
                case NORMAL:
                default:
                    multiplier = 0.5;
                    break;
            }

            double xSpeed =
                m_xLimiter.calculate(
                    // Eixo Y esquerdo vira avanço/recuo; sinal negativo adapta o joystick ao robô.
                    // applyDeadband zera valores pequenos para compensar folga física do controle.
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(),
                        OIConstants.kDriveDeadband)) * multiplier;

            double ySpeed =
                m_yLimiter.calculate(
                    // Eixo X esquerdo controla deslocamento lateral do swerve.
                    // Em swerve, o robô consegue andar de lado sem girar o chassi.
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(),
                        OIConstants.kDriveDeadband)) * multiplier;

            double rotSpeed =
                m_rotLimiter.calculate(
                    // Eixo X direito controla rotação do robô.
                    // O limitador evita mudanças bruscas de rotação que dificultam pilotagem.
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(),
                        OIConstants.kDriveDeadband)) * multiplier;

            // ================= MODO VISÃO (aproxima e depois vibra) =================
            if (visionState != VisionState.OFF) {

                // Se não tem tag, para o robô e espera a tag voltar
                // return encerra a execução desta lambda neste ciclo, impedindo cair no drive manual.
                if (!hasValidTag) {
                    m_robotDrive.drive(0.0, 0.0, 0.0, false);
                    inRangeCounter = 0;
                    return;
                }

                // Filtro simples para reduzir ruído (EMA)
                if (!filterInit) {
    // Na primeira leitura, inicializa o filtro com a medição real para evitar salto artificial.
    filteredDistance = tagDistance;
    filterInit = true;
} else {
    // EMA: mistura leitura antiga e nova para reduzir ruído sem atrasar demais a resposta.
    filteredDistance = 0.75 * filteredDistance + 0.25 * tagDistance;
}

// Erro positivo significa que a câmera está mais longe que a distância alvo.
double error = filteredDistance - TARGET_DIST;

// Telemetria para calibrar distância, tolerâncias e ganhos durante testes no campo.
// putNumber/putString enviam valores para o SmartDashboard sem afetar o comportamento do robô.
SmartDashboard.putNumber("TagDistanceRaw", tagDistance);
SmartDashboard.putNumber("TagDistanceFiltered", filteredDistance);
SmartDashboard.putNumber("TagError", error);
SmartDashboard.putString("VisionState", visionState.toString());

// ---------- VIBRATE ----------
if (visionState == VisionState.VIBRATE) {
    // Depois de alinhar, o robô oscila lateralmente como feedback visível ao operador.
    // Math.sin gera um valor entre -1 e 1; multiplicar por VIB_SPEED define a amplitude.
    double t = Timer.getFPGATimestamp() - vibTime;
    double vib = Math.sin(t * VIB_FREQ) * VIB_SPEED;

    m_robotDrive.drive(0.0, vib, 0.0, false);
    return;
}

// ---------- APPROACH ----------
if (visionState == VisionState.APPROACH) {

    // Distância usa a medição filtrada; ângulo usa yaw normalizado para -180 a 180 graus.
    double distError = filteredDistance - TARGET_DIST;
    double angleError = MathUtil.inputModulus(tagAngle, -180, 180);

    // ===== AJUSTE ÂNGULO + DISTÂNCIA (SEM LATERAL) =====

    double rot = 0;
    double x = 0;

    // Corrige ângulo
    if (Math.abs(angleError) > ANGLE_TOL) {

        // Controle proporcional de yaw: gira até a tag ficar quase centralizada angularmente.
        // clamp limita a rotação para evitar giro agressivo perto da AprilTag.
        rot = -angleError * KP_ROT;
        rot = MathUtil.clamp(rot, -MAX_ROT_SPEED, MAX_ROT_SPEED);
    }

    // Corrige distância
    if (Math.abs(distError) > DIST_TOL) {

        // Controle proporcional de avanço/recuo: aproxima ou afasta até TARGET_DIST.
        // Como x é robot-relative neste modo, positivo/negativo dependem da convenção do drive.
        x = -distError * KP_DIST;
        x = MathUtil.clamp(x, -MAX_X_SPEED, MAX_X_SPEED);
    }

    // Se ambos já estão alinhados
    if (Math.abs(distError) <= DIST_TOL && Math.abs(angleError) <= ANGLE_TOL) {

        // Contador de estabilidade: cada ciclo dentro da tolerância soma 1.
        inRangeCounter++;

        // Exige vários ciclos dentro da tolerância para não aceitar uma leitura isolada da câmera.
        if (inRangeCounter >= STABLE_LOOPS) {
            visionState = VisionState.VIBRATE;
            vibTime = Timer.getFPGATimestamp();
        }

        m_robotDrive.drive(0, 0, 0, false);
        return;
    }

    // Move robô corrigindo os dois ao mesmo tempo
    m_robotDrive.drive(x, 0, rot, false);
    return;

        } 

        }

            // ================= DRIVE NORMAL =================
            // Quando a visão está OFF, o joystick do piloto controla o swerve normalmente.
            m_robotDrive.drive(
                xSpeed,
                ySpeed,
                rotSpeed,
                false
            );

        }, m_robotDrive)
    );
    }


  /* ================= BOTOES ================= */
  /**
   * Mapeia botões e gatilhos dos controles para comandos.
   *
   * <p>Bindings ficam centralizados aqui para facilitar treino da equipe: se um botão mudar na
   * Driver Station, este é o primeiro arquivo a conferir.
   */
  private void configureButtonBindings() {

    // Botão X do controle do shooter -> descer até zona segura (ELEVATOR_MIN - 20)
    // JoystickButton cria um gatilho digital baseado em um botão físico do XboxController.
    new JoystickButton(m_ShoterController, Button.kX.value)
        .whileTrue(
            // whileTrue mantém este RunCommand agendado enquanto o botão estiver pressionado.
            new RunCommand(() -> {

                double pos = elevatorEncoder.getPosition();

                // Se ainda está abaixo da zona segura, manda descer; caso contrário para o motor.
                if (pos < ELEVATOR_SAFE_ZONE) {
                    elevatorMotor.set(0.4); // desce
                } else {
                    elevatorMotor.stopMotor(); // chegou na zona segura
                }

            })
        )
        // onFalse executa quando o botão deixa de estar pressionado; é a parada de segurança.
        .onFalse(new InstantCommand(elevatorMotor::stopMotor));

    // BOTÃO Y -> Toggle modo vibração
    new JoystickButton(m_ShoterController, Button.kY.value)
    .onTrue(new InstantCommand(() -> {

        // Y alterna entre ligar alinhamento por visão e cancelar imediatamente esse modo.
        if (visionState == VisionState.OFF) {
            // Ao ligar, reinicia contador e filtro para não usar valores antigos de outra tentativa.
            visionState = VisionState.APPROACH;
            inRangeCounter = 0;
            filterInit = false;
            vibTime = Timer.getFPGATimestamp();
            System.out.println("Vision Mode ON");
        } else {
            // Ao desligar, zera estado e manda velocidade zero para não deixar o último comando ativo.
            visionState = VisionState.OFF;
            inRangeCounter = 0;
            filterInit = false;
            m_robotDrive.drive(0.0, 0.0, 0.0, false); // garante parar ao desligar
            System.out.println("Vision Mode OFF");
        }

    }));
    /* TRAVA EM X (RB) */
    // RB do piloto chama setX enquanto pressionado para travar/orientar as rodas do swerve.
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(
            new RunCommand(
                // Method call simples: enquanto RB está pressionado, pede posição de trava ao drive.
                () -> m_robotDrive.setX(),
                m_robotDrive
            )
        );

    /* MODO RAPIDO - B */
    // B deixa o drive com multiplicador total para deslocamentos rápidos.
    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(new InstantCommand(() -> driveMode = DriveMode.FAST));

    /* MODO NORMAL - A */
    // A volta para velocidade intermediária, usada como modo padrão de teleop.
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(new InstantCommand(() -> driveMode = DriveMode.NORMAL));

    /* MODO SUPER LENTO - X */
    // X reduz bastante a velocidade para alinhamentos finos perto de elementos de campo.
    new JoystickButton(m_driverController, Button.kX.value)
        .onTrue(new InstantCommand(() -> driveMode = DriveMode.SUPER_SLOW));

    /* RESET DO GYRO - START */
    new JoystickButton(m_driverController, Button.kStart.value)
        .onTrue(
            new InstantCommand(() -> {
              m_robotDrive.zeroHeading();
              System.out.println("Gyro ZERADO pelo botão START");
            }, m_robotDrive)
        );


        /* ================= SHOOTER / INTAKE (FUNÇÕES) ================= */

        // FUNÇÃO 1: RT (Axis 3) -> armazenar a bola
        // Enquanto RT está pressionado, shooter e intake giram no sentido de armazenar/preparar.
        // Trigger permite usar eixo analógico como botão, comparando o valor com um limiar.
        new Trigger(() -> m_ShoterController.getRawAxis(3) > 0.2)
            .whileTrue(
                new RunCommand(
                    () -> {
                        
                        shooterMotor.set(0.5);
                        intakeMotor.set(0.5);
                    }
                )
            )
            .onFalse(
                new InstantCommand(
                    () -> {
                        shooterMotor.stopMotor();
                        intakeMotor.stopMotor();
                    }
                )
            );

                    
        // LT (Axis 2) -> disparo com redução gradual da velocidade do shooter ao longo do tempo.
        // A lógica usa tempo desde o início do gatilho para reduzir a rotação do shooter em degraus.
        new Trigger(() -> m_ShoterController.getRawAxis(2) > 0.2)

        .onTrue(new InstantCommand(() -> {
            // Marca o início do disparo para calcular os degraus de velocidade.
            ltStartTime = Timer.getFPGATimestamp();
        }))

        .whileTrue(
            new RunCommand(() -> {

                double tempo = Timer.getFPGATimestamp() - ltStartTime;

                // Calcula quantos "passos" de tempo já passaram
                // Cast para int descarta a parte decimal: 0.9 s vira passo 0, 1.2 s vira passo 1.
                int step = (int)(tempo / SHOOTER_STEP_TIME);

                // Calcula nova velocidade
                // A cada passo, subtrai SHOOTER_STEP_DECREASE da velocidade inicial.
                double shooterSpeed = SHOOTER_START_SPEED - (step * SHOOTER_STEP_DECREASE);

                // Limita velocidade mínima
                // Math.max impede cair abaixo do mínimo mesmo após muito tempo segurando LT.
                shooterSpeed = Math.max(shooterSpeed, SHOOTER_MIN_SPEED);

                // Aplica velocidade
                shooterMotor.set(shooterSpeed);

                // Intake sempre disparando
                intakeMotor.set(-1);

            })
        )

        .onFalse(
            new InstantCommand(() -> {
                shooterMotor.stopMotor();
                intakeMotor.stopMotor();
            })
        );

        /*new Trigger(() -> m_ShoterController.getRawAxis(2) > 0.2)

        .whileTrue(
                new RunCommand(
                    () -> {
                        
                        shooterMotor.set(0.71);
                        intakeMotor.set(-1);
                    }
                )
            )
            .onFalse(
                new InstantCommand(
                    () -> {
                        shooterMotor.stopMotor();
                        intakeMotor.stopMotor();
                    }
                )
            );*/

            /*.debounce(0.05)

            .onTrue(new InstantCommand(() -> {
                ltStartTime = Timer.getFPGATimestamp();
            }))

            .whileTrue(new RunCommand(() -> {

                // Shooter sempre ativo enquanto LT estiver pressionado
                shooterMotor.set(0.65);

                double tempo = Timer.getFPGATimestamp() - ltStartTime;

                double cicloTotal = INTAKE_PULL_TIME + INTAKE_SHOOT_TIME;
                double ciclo = tempo % cicloTotal;

                if (ciclo < INTAKE_PULL_TIME) {
                    intakeMotor.set(0.8);      // Puxa
                } else {
                    intakeMotor.set(-0.8);     // Dispara
                }

            }))

            .onFalse(new InstantCommand(() -> {
                shooterMotor.stopMotor();
                intakeMotor.stopMotor();
            }));

*/





        // FUNÇÃO 3: LB -> DESPEJAR BOLA
        // Inverte os sentidos para ejetar a peça para fora do mecanismo sem disparar normalmente.
        // Útil para desentupir ou remover peça de jogo sem passar pela sequência de tiro.
        new JoystickButton(m_ShoterController , Button.kLeftBumper.value)
            .whileTrue(
                new RunCommand(
                    () -> {
                        
                        shooterMotor.set(-1);
                        intakeMotor.set(1);
                    }
                )
            )
            .onFalse(
                new InstantCommand(
                    () -> {
                        shooterMotor.stopMotor();
                        intakeMotor.stopMotor();
                    }
                )
            );

        /* ================= ELEVADOR ================= */      


        // Botão A do controle do shooter -> subir elevador
        // O encoder relativo impede subir além do limite máximo configurado.
        // Como ELEVATOR_MAX é negativo neste robô, subir acontece enquanto pos > ELEVATOR_MAX.
        new JoystickButton(m_ShoterController, Button.kA.value)
            .whileTrue(
                new RunCommand(() -> {
                    double pos = elevatorEncoder.getPosition();

                    if (pos > ELEVATOR_MAX) {
                        elevatorMotor.set(-0.4); // sobe
                    } else {
                        elevatorMotor.stopMotor(); // limite superior atingido
                    }
                    System.out.println("Elevator Pos: " + elevatorEncoder.getPosition());
                })
            )
            .onFalse(new InstantCommand(elevatorMotor::stopMotor));



            // Botão B do controle do shooter -> descer elevador
        // O limite inferior evita continuar forçando o motor depois de chegar perto da base.
        // Aqui o sentido positivo do motor corresponde ao movimento de descida.
        new JoystickButton(m_ShoterController, Button.kB.value)
            .whileTrue(
                new RunCommand(() -> {
                    double pos = elevatorEncoder.getPosition();

                    if (pos < ELEVATOR_MIN) {
                        elevatorMotor.set(0.4); // desce
                    } else {
                        elevatorMotor.stopMotor(); // limite inferior atingido
                    }
                    System.out.println("Elevator Pos: " + elevatorEncoder.getPosition());
                })
            )
            .onFalse(new InstantCommand(elevatorMotor::stopMotor));
  }

    /* ================= AUTÔNOMO ================= */
  /** Fornece ao Robot.java a rotina autônoma escolhida antes da partida. */
  public Command getAutonomousCommand() {

    // Retorna exatamente a opção selecionada no chooser publicado no SmartDashboard.
    // Robot.autonomousInit() agenda o Command retornado por este método.
    return autoChooser.getSelected();

    //return new PathPlannerAuto("Auto_Shoot");

    /* 
    Command selected = autoChooser.getSelected();
    System.out.println("AUTO SELECIONADO: " + selected);
    return selected;
*/
/* 
    System.out.println("AUTO INICIADO");

    return new InstantCommand(() -> {
        // ZERA O GYRO E A ODOMETRIA
        m_robotDrive.zeroHeading();
        m_robotDrive.resetOdometry(
            new Pose2d(3.0, 7.0, new Rotation2d(0))
        );
      })
      // NOME DO AUTO NO PATHPLANNER
      .andThen(new PathPlannerAuto("Teste_Auto"));
  }*/
  }
}
