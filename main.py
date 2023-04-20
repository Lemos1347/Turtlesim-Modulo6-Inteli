import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute, Spawn, Kill
from time import sleep


# Classe Turtle que contem todas as informações necessárias para uma tartaruga ser criada
class Turtle:
    amount = 0
    def __init__(self):
        self.name = f"turtle{Turtle.amount}"
        Turtle.amount += 1

# Classe Turtle_controller que contem todos os métodos necessários para controlar uma tartaruga. Essa classe é um nó do ROS para que possa se comunicar com o nó do TurtleSim
class Turtle_controller(Node):
    def __init__(self):
        # Criando um nó no ROS com o nome "turtle_controller"
        super().__init__('turtle_controller')
        sleep(0.5)
        # Limpando a tela do TurtleSim
        self._kill_first_turtle()

    # Metodo para uso interno da classe para matar a primeira tartaruga criada
    def _kill_first_turtle(self):
        # Utilizando um método da classe herdada para utilizar o serviço kill do TurtleSim
        kill_client = self.create_client(Kill, 'kill')
        # Configurando a chamada do serviço
        kill_request = Kill.Request()
        kill_request.name = 'turtle1'
        # Chamando o serviço
        kill_client.call_async(kill_request)
        sleep(1)
    
    # Metodo para mover uma tartaruga
    def move_turtle(self, turtle:Turtle, x=None, y=None, z=None):
        # Utilizando um método da classe herdada para conseguir publicar no tópico cmd_vel do TurtleSim
        move_publisher = self.create_publisher(Twist, f'{turtle.name}/cmd_vel', 10)
        # Criando uma mensagem do tipo Twist (o que vai ser publicado)
        twist_msg = Twist()
        # Realizando verificações para modificar a mensagem que vai ser passada de acordo com os parâmetros requisitados
        if x:
            twist_msg.linear.x = x
        if y:
            twist_msg.linear.y = y
        if z:
            twist_msg.angular.z = z
        # Publicando a mensagem no tópico
        move_publisher.publish(twist_msg)
        sleep(1)
    
    # Metodo para mudar a cor do rastro de um tartaruga
    def change_color(self, turtle:Turtle, r=0, g=0, b=0, width=0, off=0):
        # Utilizando um método da classe herdada para utilizar o serviço set_pen do TurtleSim
        pen_client = self.create_client(SetPen, f'{turtle.name}/set_pen')
        # Configurando a chamada do serviço
        pen_request = SetPen.Request()
        pen_request.r = r
        pen_request.g = g
        pen_request.b = b
        pen_request.width = width
        pen_request.off = off
        # Chamando o serviço
        pen_client.call_async(pen_request)
        sleep(1)

    # Metodo para matar uma tartaruga
    def kill_turtle(self, turtle:Turtle):
        # Utilizando um método da classe herdada para utilizar o serviço kill do TurtleSim
        kill_client = self.create_client(Kill, 'kill')
        # Configurando a chamada do serviço
        kill_request = Kill.Request()
        kill_request.name = turtle.name
        # Chamando o serviço
        kill_client.call_async(kill_request)
        sleep(1)
    
    # Metodo para criar um tartaruga
    def spawn_turtle(self, turtle:Turtle, x=None, y=None, theta=None):
        # Utilizando um método da classe herdada para utilizar o serviço spawn do TurtleSim
        spawn_client = self.create_client(Spawn, 'spawn')
        # Configurando a chamada do serviço
        spawn_request = Spawn.Request()
        # Realizando verificações para modificar a mensagem que vai ser passada de acordo com os parâmetros requisitados
        if x:
            spawn_request.x = x
        if y:
            spawn_request.y = y
        if theta:
            spawn_request.theta = theta
        spawn_request.name = turtle.name
        # Chamando o serviço
        spawn_client.call_async(spawn_request)
        sleep(1)       


# Funcao para limpar a tela inicial do TurtleSim
def clean_screen(turtle_controller: Turtle_controller):
    turtle = Turtle()
    turtle_controller.spawn_turtle(turtle, x=0.0, y=0.0)
    turtle_controller.change_color(turtle, r=255, g=255, b=255, width=100)
    turtle_controller.move_turtle(turtle, x=11.0)
    turtle_controller.move_turtle(turtle, y=2.0)
    turtle_controller.move_turtle(turtle, x=-11.0)
    turtle_controller.move_turtle(turtle, y=2.0)
    turtle_controller.move_turtle(turtle, x=11.0)
    turtle_controller.move_turtle(turtle, y=2.0)
    turtle_controller.move_turtle(turtle, x=-11.0)
    turtle_controller.move_turtle(turtle, y=2.0)
    turtle_controller.move_turtle(turtle, x=11.0)
    turtle_controller.move_turtle(turtle, y=2.2)
    turtle_controller.move_turtle(turtle, x=-11.0)
    turtle_controller.kill_turtle(turtle)
    sleep(0.5)

# Funcao para desenhar a primeira montanha no TurtleSim
def draw_first_mountain(turtle_controller: Turtle_controller):
    turtle = Turtle()
    turtle_controller.spawn_turtle(turtle, x=0.0, y=2.0)
    turtle_controller.change_color(turtle, r=150, g=121, b=105, width=5)
    turtle_controller.move_turtle(turtle, z=1.55)
    turtle_controller.move_turtle(turtle, x=15.0, z = -3.0, y=-2.0)
    turtle_controller.move_turtle(turtle, z=-1.55)
    sleep(1)
    turtle_controller.kill_turtle(turtle)

# Funcao para desenhar a segunda montanha no TurtleSim
def draw_second_mountain(turtle_controller: Turtle_controller):
    turtle = Turtle()
    turtle_controller.spawn_turtle(turtle, x=8.5, y=4.7)
    turtle_controller.change_color(turtle, r=150, g=121, b=105, width=5)
    turtle_controller.move_turtle(turtle, x=20.0, z = -2.0, y=2.0)
    sleep(1)
    turtle_controller.kill_turtle(turtle)

# Funcao para desenhar o chao no TurtleSim
def draw_floor(turtle_controller: Turtle_controller):
    turtle = Turtle()
    turtle_controller.spawn_turtle(turtle, x=0.0, y=1.0)
    turtle_controller.change_color(turtle, r=0, g=255, b=0, width=100)
    turtle_controller.move_turtle(turtle, x=11.0)
    sleep(1)
    turtle_controller.kill_turtle(turtle)

# Funcao para desenha o ceu no TurtleSim
def draw_sky(turtle_controller: Turtle_controller):
    turtle = Turtle()
    turtle_controller.spawn_turtle(turtle, x=0.0, y=10.0)
    turtle_controller.change_color(turtle, r=0, g=0, b=200, width=100)
    turtle_controller.move_turtle(turtle, x=11.0)
    sleep(1)
    turtle_controller.kill_turtle(turtle)

# Funcao para desenha o sol no TurtleSim
def draw_sun(turtle_controller: Turtle_controller):
    turtle = Turtle()
    turtle_controller.spawn_turtle(turtle, x=8.0, y=8.0)
    turtle_controller.change_color(turtle, r=255, g=255, b=0, width=50)
    # Loop para completar o circulo
    for i in range(3):
        turtle_controller.move_turtle(turtle, x=2.0, z=2.0)
    turtle_controller.move_turtle(turtle, z=1.55)
    turtle_controller.move_turtle(turtle, x=1.0)
    sleep(1)
    turtle_controller.kill_turtle(turtle)

# Funcão principal de execução
def main(args=None):
    # Inicializando o ROS
    rclpy.init(args=args)
    # Instanciando um objeto Turtle_controller
    turtle_controller = Turtle_controller()

    # Chamando as funções para desenhar no TurtleSim
    clean_screen(turtle_controller)
    draw_first_mountain(turtle_controller)
    draw_second_mountain(turtle_controller)
    draw_floor(turtle_controller)
    draw_sky(turtle_controller)
    draw_sun(turtle_controller)

    # Finalizando o ROS e finalizando o nó criado para comunicar com o nó do TurtleSim
    turtle_controller.destroy_node()
    rclpy.shutdown()

# Ponto de inicialização do programa
if __name__ == '__main__':
    main()
