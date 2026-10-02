package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * Comando autônomo que usa PhotonVision para aproximar o robô de uma AprilTag específica.
 *
 * <p>Ele procura somente as tags configuradas, corrige primeiro o erro lateral e depois a distância.
 * Isso reduz movimentos diagonais durante alinhamento e deixa o comportamento mais previsível.
 *
 * <p>Como herda de {@code Command}, a WPILib chama automaticamente {@code initialize()} uma vez,
 * {@code execute()} a cada ciclo, {@code isFinished()} para saber se terminou e {@code end()} ao
 * encerrar ou ser interrompido.
 */
public class VisionAlignStation extends Command {

    // Subsistema de drive que receberá as velocidades calculadas pelo alinhamento.
    // "final" indica que a referência para o DriveSubsystem não será trocada depois do construtor.
    private final DriveSubsystem drive;

    // Câmera PhotonVision usada para localizar AprilTags no campo.
    private final PhotonCamera camera;

    // TAGS possíveis para esta rotina. Outras tags vistas pela câmera são ignoradas.
    // Fiducial ID é o número gravado no mapa de AprilTags do campo.
    private static final int TAG_A = 10;
    private static final int TAG_B = 20;

    // POSIÇÃO DESEJADA em relação à câmera: distância frontal e deslocamento lateral.
    // TARGET_DIST usa metros; TARGET_Y usa o eixo lateral da transformação câmera -> tag.
    private static final double TARGET_DIST = 1.52;
    private static final double TARGET_Y = 0.287;

    // Tolerâncias aceitas antes de considerar cada eixo alinhado.
    // Valores pequenos demais podem fazer o robô oscilar; grandes demais deixam o alinhamento impreciso.
    private static final double DIST_TOL = 0.08;
    private static final double Y_TOL = 0.05;

    // Ganhos proporcionais: transformam erro de posição em velocidade de correção.
    private static final double KP_DIST = 0.5;
    private static final double KP_Y = 0.8;

    // Limites de velocidade para manter a aproximação por visão lenta e controlável.
    private static final double MAX_X = 0.12;
    private static final double MAX_Y = 0.15;

    // Evita que o comando fique preso para sempre se a câmera perder a tag ou o robô não alinhar.
    private static final double TIMEOUT_SECONDS = 6.0;

    // Timestamp salvo no início para calcular timeout.
    private double startTime;

    public VisionAlignStation(DriveSubsystem drive, PhotonCamera camera) {
        // Construtor recebe dependências prontas do RobotContainer em vez de criar outro drive/câmera.
        // Isso mantém uma única instância de cada hardware no projeto.
        this.drive = drive;
        this.camera = camera;

        // Requer o drive para impedir que outro comando controle o swerve ao mesmo tempo.
        addRequirements(drive);
    }

    @Override
    public void initialize() {

        // Marca quando a rotina começou; usado por isFinished() para encerrar por tempo.
        startTime = Timer.getFPGATimestamp();

    }

    @Override
    public void execute() {

        // getLatestResult() captura a leitura mais recente processada pelo PhotonVision.
        // Esse resultado pode conter zero, uma ou várias tags detectadas no mesmo frame.
        PhotonPipelineResult result = camera.getLatestResult();

        // Sem alvo, o comportamento seguro é parar o drive e esperar outro ciclo.
        if (!result.hasTargets()) {
            drive.drive(0,0,0,false);
            return;
        }

        PhotonTrackedTarget target = null;

        // Procura a primeira tag que pertença a esta rotina; tags de outras áreas são ignoradas.
        // O loop for percorre todos os alvos retornados pela câmera neste frame.
        for (PhotonTrackedTarget t : result.getTargets()) {

            int id = t.getFiducialId();

            // if compara o ID lido com os IDs esperados; "||" significa OU lógico.
            if (id == TAG_A || id == TAG_B) {
                target = t;
                break;
            }
        }

        // Se a câmera viu tags, mas nenhuma das tags desejadas, o robô não tenta alinhar.
        if (target == null) {
            drive.drive(0,0,0,false);
            return;
        }

        // Usa a transformação câmera -> tag para estimar distância total e erro lateral.
        // getNorm() calcula o comprimento do vetor 3D, ou seja, a distância espacial até a tag.
        double distance = target.getBestCameraToTarget().getTranslation().getNorm();
        double y = target.getBestCameraToTarget().getY();

        // Erros positivos/negativos indicam para qual lado ou distância o robô precisa corrigir.
        double distError = distance - TARGET_DIST;
        double yError = y - TARGET_Y;

        // ===== PRIMEIRO ALINHA LATERAL =====
        if (Math.abs(yError) > Y_TOL) {

            // Controle proporcional simples, limitado para não dar trancos no swerve.
            // Sinal negativo: se o erro está para um lado, o comando empurra o robô para o lado oposto.
            double ySpeed = -yError * KP_Y;
            ySpeed = MathUtil.clamp(ySpeed,-MAX_Y,MAX_Y);

            // drive(x, y, rot, fieldRelative). Aqui só mexe no eixo lateral, sem avançar ou girar.
            drive.drive(0,ySpeed,0,false);
            return;
        }

        // ===== DEPOIS AJUSTA DISTÂNCIA =====
        if (Math.abs(distError) > DIST_TOL) {

            // Depois que o lateral está aceitável, aproxima/afasta até a distância alvo.
            double xSpeed = -distError * KP_DIST;
            xSpeed = MathUtil.clamp(xSpeed,-MAX_X,MAX_X);

            // Aqui só mexe em X, porque o Y já passou pela tolerância acima.
            drive.drive(xSpeed,0,0,false);
            return;
        }

        // Alinhado: se lateral e distância estão dentro da tolerância, mantém o robô parado.
        drive.drive(0,0,0,false);
    }

    @Override
    public boolean isFinished() {

        // A rotina encerra apenas por timeout; ao alinhar ela para o drive, mas permanece ativa.
        // Timer.getFPGATimestamp() é o relógio oficial da FPGA/roboRIO, indicado para código FRC.
        if (Timer.getFPGATimestamp() - startTime > TIMEOUT_SECONDS) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        // Garante que o robô não continue com a última velocidade ao terminar ou ser interrompido.
        drive.drive(0,0,0,false);

    }
}
