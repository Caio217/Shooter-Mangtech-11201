// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 *
 * <p>Para os alunos: este arquivo é o ponto de entrada Java. Em programas comuns, o método
 * {@code main} costuma conter a lógica principal. Em FRC, ele só entrega o controle para a WPILib,
 * que passa a chamar os métodos de {@link Robot} conforme o modo do robô.
 */
public final class Main {
  // Classe utilitária: não deve ser instanciada nem conter estado do robô.
  // O construtor private impede que alguém faça "new Main()" por engano.
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    // Entrega o controle para a WPILib, que cria e executa a classe Robot no roboRIO/simulação.
    // "Robot::new" é uma referência de construtor: a WPILib recebe a receita para criar Robot.
    RobotBase.startRobot(Robot::new);
  }
}
