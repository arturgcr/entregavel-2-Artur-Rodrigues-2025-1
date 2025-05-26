import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from example_interfaces.srv import SetBool

# Classe que representa o nó responsável por calcular e publicar a média de temperatura
class MonitoramentoTemperatura(Node):
    def __init__(self):
        # Inicializa o nó com o nome usado no ROS 2
        super().__init__('temperature_monitor')

        # Assina o tópico onde os valores de temperatura são publicados
        self.subscription = self.create_subscription(
            Float64,
            'temperature',
            self.receber_temperatura,
            10
        )

        # publicador da média calculada no tópico /average_temperature
        self.publisher = self.create_publisher(
            Float64,
            'average_temperature',
            10
        )

        # Criação do serviço para reiniciar a média (reset)
        self.service = self.create_service(
            SetBool,
            'reset_average',
            self.zerar_media
        )

        # Lista que guarda as últimas 5 temperaturas
        self.temperaturas = []

        self.get_logger().info('Nó de monitoramento iniciado.')

    # Callback chamado ao receber uma nova temperatura
    def receber_temperatura(self, msg):
        self.temperaturas.append(msg.data)

        # Garante que a lista só tenha no máximo 5 valores
        if len(self.temperaturas) > 5:
            self.temperaturas.pop(0)

        # Calcula a média das temperaturas
        media = sum(self.temperaturas) / len(self.temperaturas)

        # cria e publica a mensagem com a média
        msg_media = Float64()
        msg_media.data = media
        self.publisher.publish(msg_media)

        self.get_logger().info(f'Média publicada: {media:.2f} °C')

    # Callback chamado quando o serviço de reset é acionado
    def zerar_media(self, request, response):
        if request.data:  # Se o cliente enviou 
            self.temperaturas = []
            self.get_logger().info('Lista de temperaturas foi zerada.')
            response.success = True
            response.message = 'Média reiniciada com sucesso.'
        else:
            response.success = False
            response.message = 'Requisição ignorada.'
        return response

# Função principal que inicializa o nó e o mantém em execução
def main(args=None):
    rclpy.init(args=args)
    no = MonitoramentoTemperatura()
    rclpy.spin(no)
    no.destroy_node()
    rclpy.shutdown()

# Executa a função main se o script for chamado diretamente
if __name__ == '__main__':
    main()
