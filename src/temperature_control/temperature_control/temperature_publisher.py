import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import random

# Classe do nó responsável por publicar valores de temperatura simulados
class PublicadorTemperatura(Node):
    def __init__(self):
        # inicializa o nó com o nome reconhecido pelo ROS
        super().__init__('temperature_publisher')

        # Cria um publicador no tópico /temperature, com tipo Float64
        self.publisher_ = self.create_publisher(Float64, 'temperature', 10)

        # Define um timer que chama a função de publicção a cada 2 Hz
        self.timer = self.create_timer(0.5, self.publicar_temperatura)

        self.get_logger().info('Nó de publicação de temperatura iniciado.')

    # Função que gera um valor aleatório e publica no topico
    def publicar_temperatura(self):
        numero_aleatorio = random.randint(80, 100)
        temperatura = 25.0 + 5.0 * math.sin(numero_aleatorio / 10.0)

        msg = Float64()
        msg.data = temperatura
        self.publisher_.publish(msg)

        self.get_logger().info(f'Temperatura publicada: {temperatura:.2f} °C')

# Função principal que inicia o no
def main(args=None):
    rclpy.init(args=args)
    no = PublicadorTemperatura()
    rclpy.spin(no)
    no.destroy_node()
    rclpy.shutdown()

# Executa a função se o script for executado diretamente
if __name__ == '__main__':
    main()
