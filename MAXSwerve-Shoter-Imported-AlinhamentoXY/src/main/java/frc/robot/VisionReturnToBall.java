package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * Comando autônomo de retorno/alinhamento usando AprilTags vistas pela PhotonVision.
 *
 * <p>A rotina é parecida com {@link VisionAlignStation}, mas usa outro par de tags e outra distância
 * alvo. O código controla apenas o swerve; shooter, intake e elevador ficam fora deste comando.
 *
 * <p>Este arquivo é um bom exemplo de reaproveitamento de lógica: a estrutura é a mesma de outro
 * comando de visão, mas as constantes de tags e posição alvo mudam para outra situação do robô.
 */
public class VisionReturnToBall extends Command {

    // Drive swerve que será comandado durante a aproximação por visão.
    // Guardar o subsistema em campo permite usar o mesmo objeto em todos os ciclos execute().
    private final DriveSubsystem drive;

    // Câmera configurada no PhotonVision para detectar AprilTags.
    private final PhotonCamera camera;

    // Duas tags possíveis para esta rotina; qualquer outra tag detectada é ignorada.
    // IDs diferentes indicam que esta rotina procura outro local ou referência no campo.
    private static final int TAG_A = 14;
    private static final int TAG_B = 29;

    // Posição desejada da câmera em relação à tag quando o robô parar.
    // TARGET_Y = 0.0 significa tentar ficar centralizado lateralmente com a tag.
    private static final double TARGET_DIST = 0.55;
    private static final double TARGET_Y = 0.0;

    // Margem de erro aceitável para distância e deslocamento lateral.
    private static final double DIST_TOL = 0.07;
    private static final double Y_TOL = 0.05;

    // Ganhos proporcionais usados para transformar erro da visão em comando de velocidade.
    private static final double KP_DIST = 0.5;
    private static final double KP_Y = 0.8;

    // Limites de velocidade para manter a correção de visão suave.
    private static final double MAX_X = 0.12;
    private static final double MAX_Y = 0.15;

    // Timeout de segurança: encerra a rotina mesmo que o alinhamento não seja alcançado.
    private static final double TIMEOUT_SECONDS = 6.0;

    // Tempo inicial usado para calcular o timeout.
    private double startTime;

    public VisionReturnToBall(DriveSubsystem drive, PhotonCamera camera) {
        // O RobotContainer injeta as dependências. Isso evita criar duas câmeras ou dois drives.
        this.drive = drive;
        this.camera = camera;

        // Enquanto este comando roda, ele tem posse exclusiva do DriveSubsystem.
        addRequirements(drive);
    }

    @Override
    public void initialize() {

        // Reinicia a contagem de tempo sempre que o comando é agendado pelo PathPlanner.
        startTime = Timer.getFPGATimestamp();

    }

    @Override
    public void execute() {

        // Cada chamada de execute() recalcula o movimento com base na última imagem disponível.
        // Assim o robô corrige o caminho continuamente, não apenas uma vez.
        PhotonPipelineResult result = camera.getLatestResult();

        // Se nenhuma tag foi detectada, para o robô para evitar movimento sem referência.
        if (!result.hasTargets()) {
            drive.drive(0,0,0,false);
            return;
        }

        PhotonTrackedTarget target = null;

        // Seleciona apenas uma das tags esperadas para esta rotina.
        // O tipo PhotonTrackedTarget guarda informações como ID, yaw e transformações 3D.
        for (PhotonTrackedTarget t : result.getTargets()) {

            int id = t.getFiducialId();

            // Quando encontra a primeira tag válida, salva em target e sai do loop com break.
            if (id == TAG_A || id == TAG_B) {
                target = t;
                break;
            }
        }

        // Tags de outras áreas do campo não são usadas para este alinhamento.
        if (target == null) {
            drive.drive(0,0,0,false);
            return;
        }

        // A transformação câmera -> tag fornece a distância total e o deslocamento Y lateral.
        // A câmera mede no referencial dela; por isso estes valores dependem da montagem da câmera.
        double distance = target.getBestCameraToTarget().getTranslation().getNorm();
        double y = target.getBestCameraToTarget().getY();

        // Calcula o erro em relação ao ponto desejado antes de aplicar controle proporcional.
        double distError = distance - TARGET_DIST;
        double yError = y - TARGET_Y;

        // ===== PRIMEIRO ALINHA LATERAL =====
        if (Math.abs(yError) > Y_TOL) {

            // Corrige apenas o eixo lateral até ficar dentro da tolerância.
            // MathUtil.clamp impede que um erro grande gere velocidade maior que MAX_Y.
            double ySpeed = -yError * KP_Y;
            ySpeed = MathUtil.clamp(ySpeed,-MAX_Y,MAX_Y);

            drive.drive(0,ySpeed,0,false);
            return;
        }

        // ===== DEPOIS AJUSTA DISTÂNCIA =====
        if (Math.abs(distError) > DIST_TOL) {

            // Corrige a distância depois do alinhamento lateral para simplificar a trajetória.
            // O ganho KP_DIST define o quanto o robô reage para cada metro de erro.
            double xSpeed = -distError * KP_DIST;
            xSpeed = MathUtil.clamp(xSpeed,-MAX_X,MAX_X);

            drive.drive(xSpeed,0,0,false);
            return;
        }

        // Alinhado: dentro das duas tolerâncias, o drive recebe zero em todos os eixos.
        drive.drive(0,0,0,false);
    }

    @Override
    public boolean isFinished() {

        // Este comando termina por tempo; se alinhar antes, execute() mantém o drive parado.
        // Retornar false mantém o comando ativo; retornar true chama end(false).
        if (Timer.getFPGATimestamp() - startTime > TIMEOUT_SECONDS) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        // Para o swerve tanto no fim normal quanto em interrupções por outro comando.
        drive.drive(0,0,0,false);

    }
}
