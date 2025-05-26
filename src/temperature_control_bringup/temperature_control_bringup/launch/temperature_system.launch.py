from launch import LaunchDescription
from launch_ros.actions import Node

# Função obrigatória que retorna os nós que serão executados
def generate_launch_description():
    # Nó responsável por publicar valores de temperatura no tópico /temperature
    publisher_node = Node(
        package='temperature_control',        # Pacote onde está o nó
        executable='temperature_publisher',   # Nome do executável registrado no setup
        name='temperature_publisher'          # Nome do nó no sistema ROS
    )

    # Nó responsável por calcular a média e oferecer o servico de reset
    monitor_node = Node(
        package='temperature_control',
        executable='temperature_monitor',
        name='temperature_monitor'
    )

    # Nó responsável por exibir no terminal os valores da média publicados
    avg_monitor_node = Node(
        package='temperature_control',
        executable='temperature_avg_monitor',
        name='temperature_avg_monitor'
    )

    # Retorna todos os nós que devem ser executados quando for iniciad
    return LaunchDescription([
        publisher_node,
        monitor_node,
        avg_monitor_node
    ])
