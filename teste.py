import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute, Spawn, Kill
from time import sleep


class Turtle_working(Node):
    turtle_ammount = 0
    def __init__(self):
        self.turtle_name = f"turtle{Turtle_working.turtle_ammount}"
        Turtle_working.turtle_ammount += 1
        # Criando nome do nó no ros
        super().__init__('turtle_controller')

        # Criando um publisher para escrever no tópico turtle1/cmd_vel
        # Twist é o tipo de mensagem que será publicada
        # Twist tem a seguinte composicao:
        #   linear --> x, y, z
        #   angular --> x, y, z
        self.move_publisher_ = self.create_publisher(Twist, f'{self.turtle_name}/cmd_vel', 10)

        # Criando um cliente para escrever no serviço turtle1/set_pen
        self.pen_client = self.create_client(SetPen, f'{self.turtle_name}/set_pen')

        # Criando um cliente para escrever no serviço turtle1/teleport_absolute
        self.teleport_client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')

        #Criando um cliente para escrever no serviço spawn
        self.spawn_client = self.create_client(Spawn, 'spawn')

        # Criando um cliente para escrever no serviço kill
        self.kill_client = self.create_client(Kill, 'kill')

        # Criando um timer para executar a funcao move_turtle a cada 0.1 segundos
        #self.timer_ = self.create_timer(0.1, self.move_turtle)

        self.spawn_turtle(x=2.0, y=7.0)
        self.change_color(r=255, g=0, b=0, width=3)
        self.move_turtle(x=10.0, y=0.0)
        self.kill_turtle()


    def move_turtle(self, x=None, y=None, z=None):
        # Criando uma mensagem do tipo Twist (o que vai ser publicado)
        self.twist_msg_ = Twist()

        if x:
            self.twist_msg_.linear.x = x
        if y:
            self.twist_msg_.linear.y = y
        if z:
            self.twist_msg_.angular.z = z
        self.move_publisher_.publish(self.twist_msg_)
        sleep(1)
    
    def change_color(self, r=0, g=0, b=0, width=0, off=0):
        pen_request = SetPen.Request()
        pen_request.r = r
        pen_request.g = g
        pen_request.b = b
        pen_request.width = width
        pen_request.off = off
        self.pen_client.call_async(pen_request)
        sleep(1)
    
    def teleport_turtle(self, x=0, y=0):
        teleport_request = TeleportAbsolute.Request()
        # De -7.0 ate 10.0
        teleport_request.x = 4.0
        # De 1 ate 11
        teleport_request.y = 7.0
        self.teleport_client.call_async(teleport_request)
        sleep(1)

    def kill_turtle(self):
        kill_request = Kill.Request()
        kill_request.name = self.turtle_name
        self.kill_client.call_async(kill_request)
        sleep(1)
    
    def spawn_turtle(self, x=None, y=None, theta=None):
        spawn_request = Spawn.Request()
        if x:
            spawn_request.x = x
        if y:
            spawn_request.y = y
        if theta:
            spawn_request.theta = theta
        spawn_request.name = self.turtle_name
        self.spawn_client.call_async(spawn_request)
        sleep(1)

def main_working(args=None):
    rclpy.init(args=args)
    turtle = Turtle_working()
    #rclpy.spin(turtle)
    turtle.destroy_node()
    rclpy.shutdown()









class Turtle:
    amount = 0
    def __init__(self):
        self.name = f"turtle{Turtle.amount}"
        Turtle.amount += 1


class Turtle_controller(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        sleep(0.5)
        self._kill_first_turtle()

    def _kill_first_turtle(self):
        kill_client = self.create_client(Kill, 'kill')
        kill_request = Kill.Request()
        kill_request.name = 'turtle1'
        kill_client.call_async(kill_request)
        sleep(1)
    
    def move_turtle(self, turtle:Turtle, x=None, y=None, z=None):
        move_publisher = self.create_publisher(Twist, f'{turtle.name}/cmd_vel', 10)
        # Criando uma mensagem do tipo Twist (o que vai ser publicado)
        twist_msg = Twist()

        if x:
            twist_msg.linear.x = x
        if y:
            twist_msg.linear.y = y
        if z:
            twist_msg.angular.z = z
        move_publisher.publish(twist_msg)
        sleep(1)
    
    def change_color(self, turtle:Turtle, r=0, g=0, b=0, width=0, off=0):
        pen_client = self.create_client(SetPen, f'{turtle.name}/set_pen')
        pen_request = SetPen.Request()
        pen_request.r = r
        pen_request.g = g
        pen_request.b = b
        pen_request.width = width
        pen_request.off = off
        pen_client.call_async(pen_request)
        sleep(1)

    def kill_turtle(self, turtle:Turtle):
        kill_client = self.create_client(Kill, 'kill')
        kill_request = Kill.Request()
        kill_request.name = turtle.name
        kill_client.call_async(kill_request)
        sleep(1)
    
    def spawn_turtle(self, turtle:Turtle, x=None, y=None, theta=None):
        spawn_client = self.create_client(Spawn, 'spawn')
        spawn_request = Spawn.Request()
        if x:
            spawn_request.x = x
        if y:
            spawn_request.y = y
        if theta:
            spawn_request.theta = theta
        spawn_request.name = turtle.name
        spawn_client.call_async(spawn_request)
        sleep(1)       



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

def draw_first_mountain(turtle_controller: Turtle_controller):
    turtle = Turtle()
    turtle_controller.spawn_turtle(turtle, x=0.0, y=2.0)
    turtle_controller.change_color(turtle, r=150, g=121, b=105, width=5)
    turtle_controller.move_turtle(turtle, z=1.55)
    turtle_controller.move_turtle(turtle, x=15.0, z = -3.0, y=-2.0)
    turtle_controller.move_turtle(turtle, z=-1.55)
    sleep(1)
    turtle_controller.kill_turtle(turtle)

def draw_second_mountain(turtle_controller: Turtle_controller):
    turtle = Turtle()
    turtle_controller.spawn_turtle(turtle, x=8.5, y=4.7)
    turtle_controller.change_color(turtle, r=150, g=121, b=105, width=5)
    turtle_controller.move_turtle(turtle, x=20.0, z = -2.0, y=2.0)
    sleep(1)
    turtle_controller.kill_turtle(turtle)

def draw_floor(turtle_controller: Turtle_controller):
    turtle = Turtle()
    turtle_controller.spawn_turtle(turtle, x=0.0, y=1.0)
    turtle_controller.change_color(turtle, r=0, g=255, b=0, width=100)
    turtle_controller.move_turtle(turtle, x=11.0)
    sleep(1)
    turtle_controller.kill_turtle(turtle)

def draw_sky(turtle_controller: Turtle_controller):
    turtle = Turtle()
    turtle_controller.spawn_turtle(turtle, x=0.0, y=10.0)
    turtle_controller.change_color(turtle, r=0, g=0, b=200, width=100)
    turtle_controller.move_turtle(turtle, x=11.0)
    sleep(1)
    turtle_controller.kill_turtle(turtle)

def draw_sun(turtle_controller: Turtle_controller):
    turtle = Turtle()
    turtle_controller.spawn_turtle(turtle, x=8.0, y=8.0)
    turtle_controller.change_color(turtle, r=255, g=255, b=0, width=50)
    for i in range(3):
        turtle_controller.move_turtle(turtle, x=2.0, z=2.0)
    turtle_controller.move_turtle(turtle, z=1.55)
    turtle_controller.move_turtle(turtle, x=1.0)
    sleep(1)
    turtle_controller.kill_turtle(turtle)

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = Turtle_controller()

    clean_screen(turtle_controller)
    draw_first_mountain(turtle_controller)
    draw_second_mountain(turtle_controller)
    draw_floor(turtle_controller)
    draw_sky(turtle_controller)
    draw_sun(turtle_controller)

    turtle_controller.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
