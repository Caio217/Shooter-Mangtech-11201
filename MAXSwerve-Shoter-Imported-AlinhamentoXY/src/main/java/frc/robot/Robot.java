// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 *
 * <p>Esta classe é a "linha do tempo" do robô. A WPILib chama métodos diferentes quando o robô liga,
 * fica desabilitado, entra no autônomo, entra no teleop ou no modo de teste. A lógica específica de
 * motores e botões fica no RobotContainer; aqui ficam as transições de modo.
 */
public class Robot extends TimedRobot {
  // Guarda o comando autônomo selecionado para poder agendar no autonomousInit e cancelar no teleop.
  // Command é a unidade de ação do command-based: pode representar trajetória, disparo, alinhamento etc.
  private Command m_autonomousCommand;

  // Ponto central onde subsistemas, comandos, controles e autonomous chooser são criados.
  // Este campo é um objeto: ao chamar new RobotContainer(), todos os bindings são configurados.
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // Em FRC, esta criação costuma ser o único setup pesado feito ao ligar o robô.
    // robotInit executa uma única vez depois que o programa inicia no roboRIO.
    m_robotContainer = new RobotContainer();
    
  }

  

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    // Sem esta chamada a cada 20 ms, comandos padrão, botões e periodic() dos subsistemas não rodam.
    // TimedRobot chama este método em ciclos periódicos; no FRC padrão, o ciclo é de 20 ms.
    
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  // Mantido vazio porque não há ação especial ao desabilitar além do gerenciamento padrão da WPILib.
  public void disabledInit() {}

  @Override
  // Espaço reservado para diagnósticos que poderiam rodar enquanto o robô está desabilitado.
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    
    // Busca a rotina escolhida no SmartDashboard/PathPlanner por meio do RobotContainer.
    // A seleção é feita antes da partida; aqui apenas pegamos o Command já escolhido.
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    // Agenda somente se existir seleção válida; isso evita NullPointerException antes da partida.
    // schedule() entrega o comando ao CommandScheduler, que chamará initialize/execute/end dele.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // Segurança: garante que uma trajetória ou rotina autônoma não continue disputando o drive no teleop.
    // Em competição, autonomous e teleop são modos separados; cancelar evita controle duplicado.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // O teleop é controlado por comandos/bindings no RobotContainer, então não precisa de lógica aqui.
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // Ajuda a testar mecanismos individualmente sem comandos antigos interferindo.
    // testInit executa uma vez ao entrar em modo Test na Driver Station.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
