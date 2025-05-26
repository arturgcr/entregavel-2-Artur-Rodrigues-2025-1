# Entregável 2 — Sistema de Monitoramento de Temperatura (ROS 2)

Este projeto foi desenvolvido como parte do processo seletivo da Equipe Harpia, durante a etapa de capacitação prática com ROS 2.

A proposta consiste em criar um sistema de monitoramento de temperatura utilizando nós em Python, comunicação via tópicos, serviço de reinicialização de média e um launch file integrando tudo.

## Estrutura do Projeto

O repositório contém dois pacotes principais:

- `temperature_control`: onde ficam os nós:
  - `temperature_publisher`: publica valores simulados de temperatura a 2 Hz.
  - `temperature_monitor`: calcula a média das últimas 5 temperaturas e disponibiliza um serviço para resetar esse histórico.
  - `temperature_avg_monitor`: escuta a média e imprime no terminal.

- `temperature_control_bringup`: pacote de bringup que organiza e executa os três nós com um único `launch.py`.
