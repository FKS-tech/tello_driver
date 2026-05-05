# tello_driver

Pacote ROS 2 Python para usar um DJI Tello com controle, video, telemetria e visao computacional.

O pacote ainda nao implementa voo autonomo real. A estrutura atual prepara o terreno para isso mantendo os nos existentes e os topicos usados hoje.

## Nos

- `joy_node`: assina `/joy`, aplica deadzone, envia comandos `rc` para o Tello e usa botoes para `takeoff` e `land`.
- `stream_node`: ativa o SDK/stream do Tello, abre o video UDP e publica imagens ROS.
- `telemetry_node`: escuta telemetria UDP na porta `8890` e publica dados brutos e JSON.
- `vision_node`: assina imagem do Tello, roda YOLO e publica imagem anotada e deteccoes JSON.
- `visual_servo_node`: le deteccoes visuais e calcula comando de centralizacao visual em `/tello/autonomy/cmd_vel`, sem enviar comandos diretamente ao drone.

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
| `/vision/detections` | `std_msgs/String` | assinado | `visual_servo_node` |
| `/tello/autonomy/cmd_vel` | `geometry_msgs/Twist` | publicado | `visual_servo_node` |
| `/tello/autonomy/debug` | `std_msgs/String` | publicado | `visual_servo_node` |

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
ros2 run tello_driver visual_servo_node
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

Argumentos uteis do launch:

| Argumento | Default | Descricao |
| --- | --- | --- |
| `show_preview` | `true` | Liga/desliga as janelas OpenCV do `stream_node` e `vision_node`. |
| `stream_url` | `udp://0.0.0.0:11111?fifo_size=50000000&overrun_nonfatal=1` | URL do stream UDP do Tello. |
| `model_path` | `yolov8n.pt` | Modelo YOLO usado pelo `vision_node`. |
| `enable_sdk_init` | `true` | Faz o `stream_node` enviar o comando `command` ao iniciar. |
| `enable_stream_on` | `true` | Faz o `stream_node` enviar o comando `streamon` ao iniciar. |

Para testar o launch sem drone conectado, desative os comandos SDK do `stream_node`:

```bash
ros2 launch tello_driver tello_basic.launch.py show_preview:=false enable_sdk_init:=false enable_stream_on:=false
```

O launch completo `tello_bringup.launch.py` inicia a pilha principal de uso com joystick:

- `joy/joy_node`, para ler o joystick fisico;
- `tello_driver/joy_node`, para enviar `rc`, `takeoff` e `land`;
- `stream_node`, para publicar `/tello/image_raw`;
- `vision_node`, para publicar `/vision/image_annotated` e `/vision/detections`;
- `telemetry_node`, para publicar `/tello/telemetry/raw` e `/tello/telemetry/json`.

```bash
ros2 launch tello_driver tello_bringup.launch.py
```

Argumentos uteis do bringup:

| Argumento | Default | Descricao |
| --- | --- | --- |
| `enable_stream_node` | `true` | Liga/desliga o `stream_node` no bringup. |
| `enable_vision_node` | `true` | Liga/desliga o `vision_node` no bringup. |

Exemplos:

```bash
ros2 launch tello_driver tello_bringup.launch.py enable_vision_node:=false
ros2 launch tello_driver tello_bringup.launch.py enable_stream_node:=false
ros2 launch tello_driver tello_bringup.launch.py enable_stream_node:=false enable_vision_node:=false
```

Para testar somente a cadeia visual segura, existe o launch `visual_servo_test.launch.py`. Ele inicia `stream_node`, `vision_node` e `visual_servo_node`, mas nao inicia `joy_node`. Por padrao, ele tambem deixa `enable_sdk_init` e `enable_stream_on` como `false`, entao nao envia comandos SDK ao Tello:

```bash
ros2 launch tello_driver visual_servo_test.launch.py
```

Topicos para observar o servo visual:

```bash
ros2 topic echo /tello/autonomy/cmd_vel
ros2 topic echo /tello/autonomy/debug
```

## Configuracao

Os defaults continuam declarados dentro dos nos. O arquivo `config/tello_default.yaml` existe como referencia opcional para facilitar ajustes futuros.

## Proximos passos de autonomia

Ideias planejadas, ainda nao implementadas:

- integracao segura da centralizacao visual com controle real;
- deteccao de QR Code;
- missao simplificada inspirada na Fase 4 da Flying Robot League;
- criacao futura de `mission_node` e `command_mux_node`.
