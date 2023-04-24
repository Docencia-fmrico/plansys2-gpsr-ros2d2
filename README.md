[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-8d59dc4de5201274e310e4c54b9627a8934c3b88527886e3b421487c677d23eb.svg)](https://classroom.github.com/a/j9y_86cr)
# plansys2_gpsr

Ejercicio 4 de Planificación y Sistemas Cognitivos 2023

[![main](https://github.com/Docencia-fmrico/patrolling-ros2d2/actions/workflows/main.yaml/badge.svg?branch=main)](https://github.com/Docencia-fmrico/patrolling-ros2d2/actions/workflows/main.yaml)

# Patrolling ROS2D2

## Content table
1. [Introduction](#Introduction)
2. [About](#About)
3. [Usage](#Usage)

## Introduction

The objective of this project is to make a robot (in this case a TurtleBot3 Burger) to execute a plan to order a house and giving priority to the hose owner instructions.

## About

### Planing

### World

Seeing how many problems gazebo and the code from pal robotics were giving we decided to do this project using Webots and the TurtleBot3 Burger. We also wanted to use the knowledge we have from other classes on this simulator so we built our own world:

![alt text](https://github.com/Docencia-fmrico/plansys2-gpsr-ros2d2/blob/Readme/Media/House_img.png)

We also mapped it in order to navigate it:

![alt text](https://github.com/Docencia-fmrico/plansys2-gpsr-ros2d2/blob/Readme/Media/big_house.png)


### Tests

## Usage

### Installation


## Working examples

## Instructions
En grupos de 4, haced una aplicación en ROS 2 usando PlanSys2 que use el dominio de la [Práctica 3](https://github.com/Docencia-fmrico/planning-exercise/blob/main/README.md). El robot debe poder realizar en un simulador, navegando con Nav2, goals similares a los siguientes:

(ordena_casa robot1)
(abrir_puerta puerta_principal)
(dar robot1 vaso_leche abuelita)
(dar robot1 medicina abuelita)

Puntuación (sobre 10):   
* +5 correcto funcionamiento en el robot simulado.
* +2 Readme.md bien documentado con videos.
* +2 CI con Test de los nodos BT
* -3 Warnings o que no pase los tests.
