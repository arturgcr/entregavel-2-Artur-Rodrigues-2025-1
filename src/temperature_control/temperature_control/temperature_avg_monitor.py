import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# clase que representa o nó que escuta a média de temperatura
class MediaTemperatura(Node):
    def __init__(self):
        # Inicializa o nó com o nome usado dentro do sistema ROS
        super().__init__('temperature_avg_monitor')

        # Inscreve no tópico onde a média das temperaturas será publicada
        self.subscription = self.create_subscription(
            Float64,                 # Tipo da mensagem (número com ponto flutuante)
            'average_temperature',  # Nome do tópico
            self.receber_media,     # Função chamada ao receber uma nova mensagem
            10                      # Fila de mensagens
        )

        self.get_logger().info('Nó de monitoramento de média iniciado.')

    # Função chamada sempre que uma nova média é recebida
    def receber_media(self, msg):
        self.get_logger().info(f'Média recebida: {msg.data:.2f} °C')

# Função principal: inicialza o nó e mantém ele rodando
def main(args=None):
    rclpy.init(args=args)
    no = MediaTemperatura()
    rclpy.spin(no)
    no.destroy_node()
    rclpy.shutdown()

# Executa o nóse o script for chamado diretamente
if __name__ == '__main__':
    main()
