# tello_driver

Pacote ROS 2 Python para usar um DJI Tello com controle, video, telemetria e visao computacional.

O pacote ainda nao implementa autonomia. A estrutura atual prepara o terreno para isso mantendo os nos existentes e os topicos usados hoje.

## Nos

- `joy_node`: assina `/joy`, aplica deadzone, envia comandos `rc` para o Tello e usa botoes para `takeoff` e `land`.
- `stream_node`: ativa o SDK/stream do Tello, abre o video UDP e publica imagens ROS.
- `telemetry_node`: escuta telemetria UDP na porta `8890` e publica dados brutos e JSON.
- `vision_node`: assina imagem do Tello, roda YOLO e publica imagem anotada e deteccoes JSON.

## Topicos

| Topico | Tipo | Direcao | No |
| --- | --- | --- | --- |
| `/joy` | `sensor_msgs/Joy` | assinado | `joy_node` |
| `/tello/image_raw` | `sensor_msgs/Image` | publicado | `stream_node` |
| `/tello/image_raw` | `sensor_msgs/Image` | assinado | `vision_node` |
| `/tello/telemetry/raw` | `std_msgs/String` | publicado | `telemetry_node` |
| `/tello/telemetry/json` | `std_msgs/String` | publicado | `telemetry_node` |
| `/vision/image_annotated` | `sensor_msgs/Image` | publicado | `vision_node` |
| `/vision/detections` | `std_msgs/String` | publicado | `vision_node` |

## Build

Na raiz do workspace:

```bash
colcon build --packages-select tello_driver
source install/setup.bash
```

## Execucao basica

```bash
ros2 run tello_driver stream_node
ros2 run tello_driver telemetry_node
ros2 run tello_driver vision_node
ros2 run tello_driver joy_node
```

Tambem existe um launch basico que inicia video, telemetria e visao sem iniciar o controle por joystick:

```bash
ros2 launch tello_driver tello_basic.launch.py
```

Para desligar previews OpenCV:

```bash
ros2 launch tello_driver tello_basic.launch.py show_preview:=false
```

## Configuracao

Os defaults continuam declarados dentro dos nos. O arquivo `config/tello_default.yaml` existe como referencia opcional para facilitar ajustes futuros.

## Proximos passos de autonomia

Ideias planejadas, ainda nao implementadas:

- centralizacao visual de alvo;
- deteccao de QR Code;
- missao simplificada inspirada na Fase 4 da Flying Robot League;
- criacao futura de `mission_node` e `command_mux_node`.
