# Turtlesim-Modulo6-Inteli

# Desenhando uma paisagem no TurtleSim

Este repositório é a resposta para uma atividade avaliada da faculdade Inteli. Essa atividade busca o entendimento inicial de `ROS` utilizando o TurtleSim como base. A resposta da atividade contém um arquivo Python que utiliza o ROS e o TurtleSim para desenhar uma paisagem simples com montanhas, chão, céu e sol. O código cria e controla várias tartarugas, altera suas cores e as move para criar a paisagem.

## Requisitos

- [ROS 2 (testado com Humble)](https://docs.ros.org/en/humble/Installation.html)
- [turtlesim](http://wiki.ros.org/turtlesim)

## Funcionalidades

- Limpar a tela inicial do TurtleSim
- Desenhar a primeira montanha
- Desenhar a segunda montanha
- Desenhar o chão
- Desenhar o céu
- Desenhar o sol

## Estrutura do código

O código contém duas classes principais:

1. **Turtle**: Esta classe contém todas as informações necessárias para criar uma tartaruga.
2. **Turtle_controller**: Esta classe herda de `rclpy.node.Node` e contém todos os métodos necessários para controlar uma tartaruga. É um nó do ROS para se comunicar com o nó do TurtleSim.

Além disso, o código possui várias funções para desenhar diferentes partes da paisagem no TurtleSim:

- `clean_screen()`: Limpa a tela inicial do TurtleSim.
- `draw_first_mountain()`: Desenha a primeira montanha.
- `draw_second_mountain()`: Desenha a segunda montanha.
- `draw_floor()`: Desenha o chão.
- `draw_sky()`: Desenha o céu.
- `draw_sun()`: Desenha o sol.

A função `main()` inicializa o ROS, instancia um objeto `Turtle_controller` e chama as funções para desenhar a paisagem no TurtleSim.

## Como executar

Para executar o código, siga estas etapas:

1. Instale o ROS 2 e o pacote turtlesim.
2. Clone este repositório.
3. Navegue até a pasta do repositório.
4. Execute o script Python:

```pyhton
python3 main.py
```

## Vídeo Demo

<video src="funcionamento.mov" controls="controls" style="max-width: 730px;">
</video>