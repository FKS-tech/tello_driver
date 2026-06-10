# tello_driver

Pacote ROS 2 Python para usar um DJI Tello com controle, video, telemetria, visao computacional e uma ponte segura entre comandos autonomos e RC real.

English documentation: [README.en.md](README.en.md)

## Nos

- `joy_node`: assina `/joy`, aplica deadzone, envia comandos `rc` para o Tello e usa botoes para `takeoff` e `land`.
- `stream_node`: ativa o SDK/stream do Tello, abre o video UDP e publica imagens ROS.
- `telemetry_node`: escuta telemetria UDP na porta `8890` e publica dados brutos e JSON.
- `vision_node`: assina imagem do Tello, roda YOLO e publica imagem anotada e deteccoes JSON.
- `qr_node`: detecta e le QR Codes em `/tello/image_raw`, publicando deteccoes em `/vision/qr_codes`.
- `landing_base_node`: detecta bases azul/amarelo por HSV/OpenCV em `/tello/image_raw`, publicando deteccoes em `/vision/landing_base`.
- `visual_servo_node`: le deteccoes visuais e calcula comando de centralizacao visual em `/tello/autonomy/cmd_vel`, sem enviar comandos diretamente ao drone.
- `command_mux_node`: assina comandos autonomos em `/tello/autonomy/cmd_vel`, converte `Twist` para `rc` real, e executa `takeoff`/`land`/`emergency` por topico com modo armado/desarmado.

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
| `/vision/qr_codes` | `std_msgs/String` | publicado | `qr_node` |
| `/vision/qr_codes` | `std_msgs/String` | assinado | `visual_servo_node` opcional |
| `/vision/qr_image_annotated` | `sensor_msgs/Image` | publicado | `qr_node` |
| `/vision/qr_debug` | `std_msgs/String` | publicado | `qr_node` |
| `/vision/landing_base` | `std_msgs/String` | publicado | `landing_base_node` |
| `/vision/landing_base_image_annotated` | `sensor_msgs/Image` | publicado | `landing_base_node` |
| `/vision/landing_base_debug` | `std_msgs/String` | publicado | `landing_base_node` |
| `/vision/landing_base_mask` | `sensor_msgs/Image` | publicado opcional | `landing_base_node` |
| `/tello/autonomy/cmd_vel` | `geometry_msgs/Twist` | publicado | `visual_servo_node` |
| `/tello/autonomy/cmd_vel` | `geometry_msgs/Twist` | assinado | `command_mux_node` |
| `/tello/autonomy/takeoff` | `std_msgs/Empty` | assinado | `command_mux_node` |
| `/tello/autonomy/land` | `std_msgs/Empty` | assinado | `command_mux_node` |
| `/tello/autonomy/enable` | `std_msgs/Empty` | assinado | `command_mux_node` |
| `/tello/autonomy/disable` | `std_msgs/Empty` | assinado | `command_mux_node` |
| `/tello/autonomy/stop` | `std_msgs/Empty` | assinado | `command_mux_node` |
| `/tello/autonomy/emergency` | `std_msgs/Empty` | assinado | `command_mux_node` |
| `/tello/autonomy/debug` | `std_msgs/String` | publicado | `visual_servo_node` |

## Arquitetura

O pacote separa percepcao, decisao e execucao. Os nos de visao nunca enviam
comandos diretamente ao drone; eles publicam deteccoes. O `visual_servo_node`
transforma uma deteccao visual em `cmd_vel`, e o `command_mux_node` e a unica
ponte para comandos reais do Tello na pilha autonoma.

```text
stream_node
  -> /tello/image_raw
     -> vision_node         -> /vision/detections
     -> qr_node             -> /vision/qr_codes
     -> landing_base_node   -> /vision/landing_base

visual_servo_node
  -> /tello/autonomy/cmd_vel

mission_node futuro
  -> /tello/autonomy/enable
  -> /tello/autonomy/takeoff
  -> /tello/autonomy/land

command_mux_node
  -> rc real / takeoff / land / emergency
```

## Build

Na raiz do workspace:

```bash
colcon build --packages-select tello_driver
source install/setup.bash
```

## Testes

Os testes cobrem funcoes matematicas compartilhadas e a deteccao sintetica de
base azul/amarelo sem precisar de drone conectado:

```bash
colcon test --packages-select tello_driver
colcon test-result --verbose
```

## Execucao basica

```bash
ros2 run tello_driver stream_node
ros2 run tello_driver telemetry_node
ros2 run tello_driver vision_node
ros2 run tello_driver qr_node
ros2 run tello_driver landing_base_node
ros2 run tello_driver visual_servo_node
ros2 run tello_driver command_mux_node
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

O launch autonomo seguro inicia video, telemetria, visao, QR, detector de base e `command_mux_node`, mas nao inicia joystick. Nesse launch, por padrao, o `stream_node` envia `command` e `streamon`; o `command_mux_node` nao envia `command` e assume que o SDK ja foi ativado pelo `stream_node`. Isso evita disputa entre dois nos inicializando o SDK ao mesmo tempo.

```bash
ros2 launch tello_driver tello_autonomy.launch.py show_preview:=false start_armed:=false
```

Teste seco sem drone, sem `command`/`streamon`:

```bash
ros2 launch tello_driver tello_autonomy.launch.py show_preview:=false stream_enable_sdk_init:=false enable_stream_on:=false command_mux_enable_sdk_init:=false start_armed:=false
```

Teste isolado do `command_mux_node` inicializando SDK sozinho, sem `stream_node` enviar `command`/`streamon`:

```bash
ros2 launch tello_driver tello_autonomy.launch.py show_preview:=false stream_enable_sdk_init:=false enable_stream_on:=false command_mux_enable_sdk_init:=true start_armed:=false
```

Para teste controlado, ele pode iniciar armado:

```bash
ros2 launch tello_driver tello_autonomy.launch.py show_preview:=false start_armed:=true
```

Para calibrar a mascara da base durante o launch autonomo:

```bash
ros2 launch tello_driver tello_autonomy.launch.py show_preview:=false landing_base_publish_mask:=true start_armed:=false
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

Para observar QR Codes:

```bash
ros2 run tello_driver qr_node
ros2 topic echo /vision/qr_codes
ros2 topic echo /vision/qr_debug
```

Para observar bases de pouso azul/amarelo:

```bash
ros2 run tello_driver landing_base_node
ros2 topic echo /vision/landing_base
ros2 topic echo /vision/landing_base_debug
```

O `landing_base_node` usa OpenCV, nao YOLO. Ele segmenta amarelo e azul em HSV,
limpa as mascaras com morfologia, procura contornos na mascara combinada e
publica o maior candidato valido. A deteccao segue o mesmo padrao JSON do
`vision_node`, sempre como lista:

```json
[
  {
    "class_id": -1,
    "class_name": "landing_base",
    "confidence": 0.82,
    "bbox_xyxy": [120.0, 180.0, 520.0, 430.0],
    "area_ratio": 0.18,
    "center_px": [320.0, 305.0],
    "error_norm": [0.0, 0.22],
    "frame_size": [640, 480],
    "yellow_ratio_in_bbox": 0.12,
    "blue_ratio_in_bbox": 0.55
  }
]
```

Tambem existe um launch leve para testar somente stream, telemetria e detector
de base, sem YOLO, sem QR e sem `command_mux_node`:

```bash
ros2 launch tello_driver landing_base_test.launch.py show_preview:=false
ros2 topic echo /vision/landing_base
ros2 topic hz /vision/landing_base
ros2 topic list | grep landing
```

Topicos esperados:

```text
/vision/landing_base
/vision/landing_base_debug
/vision/landing_base_image_annotated
```

Para publicar tambem a mascara combinada azul/amarelo:

```bash
ros2 launch tello_driver landing_base_test.launch.py show_preview:=false publish_mask:=true
ros2 topic echo /vision/landing_base_debug
```

Com `publish_mask:=true`, o topico `/vision/landing_base_mask` publica a mascara
BGR da segmentacao combinada. Use ele para ajustar estes parametros sem mexer no
codigo:

```bash
ros2 run tello_driver landing_base_node --ros-args \
  -p publish_mask:=true \
  -p yellow_lower_h:=20 -p yellow_upper_h:=40 \
  -p blue_lower_h:=90 -p blue_upper_h:=135
```

Se a base oficial ainda nao estiver disponivel, teste com cartolina ou tecido
azul e fita amarela. Um quadrado azul com cruz, circulo ou linhas amarelas ja e
suficiente para ajustar os limiares iniciais.

O `qr_node` tenta detectar o mesmo frame em versoes pre-processadas
(`gray`, `clahe`, `adaptive_threshold` e `upscaled`) para melhorar leituras em
casos de contraste baixo, QR distante ou pequenas perdas de qualidade. O debug
em `/vision/qr_debug` indica o metodo que funcionou e lembra apenas para debug o
ultimo QR decodificado. QR parcialmente fora da imagem ainda pode falhar; o no
nao publica QR antigo como deteccao atual e nao envia comandos ao drone.

O `visual_servo_node` pode ser ajustado para cenarios com multiplos alvos:

- `target_selection_strategy:=closest_to_center` prefere o alvo mais proximo do centro da imagem.
- `target_selection_strategy:=highest_confidence` prefere a deteccao com maior confianca.
- `target_selection_strategy:=largest_area` prefere o alvo com maior area relativa.
- `enable_target_lock:=true` tenta manter o mesmo alvo entre frames para reduzir trocas bruscas quando ha varias pessoas ou objetos validos.

Exemplo:

```bash
ros2 launch tello_driver visual_servo_test.launch.py target_class_name:=person target_selection_strategy:=closest_to_center enable_target_lock:=true show_preview:=true
```

O `qr_node` apenas detecta QR Codes. O alinhamento visual continua sendo feito pelo `visual_servo_node` ao usar `input_detection_topic:=/vision/qr_codes` e `target_class_name:=qr_code`:

```bash
ros2 run tello_driver visual_servo_node --ros-args -p input_detection_topic:=/vision/qr_codes -p target_class_name:=qr_code
```

Tambem existe um launch seguro para testar QR + servo visual sem iniciar `joy_node` e sem enviar `command`/`streamon` por padrao:

```bash
ros2 launch tello_driver qr_servo_test.launch.py show_preview:=true
```

Com drone real e stream ligado pelo `stream_node`:

```bash
ros2 launch tello_driver qr_servo_test.launch.py show_preview:=true enable_sdk_init:=true enable_stream_on:=true
```

## Comando autonomo real

O `command_mux_node` e a ponte entre decisao autonoma e comando real do Tello. Ele assina `/tello/autonomy/cmd_vel`, converte `geometry_msgs/Twist` para `send_rc(left_right, forward_back, up_down, yaw)` e zera o comando se nao receber mensagem nova dentro de `watchdog_timeout` segundos.

O `Twist` aqui nao esta em m/s. Ele usa a escala RC do Tello, de `-100` a `100`, com limites adicionais por parametro (`max_xy_speed`, `max_z_speed`, `max_yaw_speed`) para reduzir o risco de comando alto por erro de autonomia.

Mapeamento:

| Twist | Tello RC |
| --- | --- |
| `linear.x` | frente/tras |
| `linear.y` | esquerda/direita |
| `linear.z` | subir/descer |
| `angular.z` | yaw |

Nao rode `joy_node` e `command_mux_node` ao mesmo tempo como controladores ativos do drone.

Por padrao, o mux inicia desarmado (`start_armed:=false`). Desarmado, ele envia RC zero e ignora `/tello/autonomy/cmd_vel`. Depois de publicar `/tello/autonomy/enable`, o mux passa a aceitar `cmd_vel`. O `/tello/autonomy/disable` volta a ignorar comandos e manda zero; `/tello/autonomy/stop` mantem o estado armado/desarmado atual, mas zera o RC imediatamente. `Land` funciona mesmo desarmado; `takeoff` exige autonomia armada; `emergency` funciona mesmo desarmado.

Teste sem voo:

```bash
ros2 run tello_driver command_mux_node
ros2 topic pub --once /tello/autonomy/enable std_msgs/msg/Empty "{}"
ros2 topic pub --once /tello/autonomy/takeoff std_msgs/msg/Empty "{}"
ros2 topic pub /tello/autonomy/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 20.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
ros2 topic pub --once /tello/autonomy/stop std_msgs/msg/Empty "{}"
ros2 topic pub --once /tello/autonomy/land std_msgs/msg/Empty "{}"
```

Comandos de seguranca:

```bash
ros2 topic pub --once /tello/autonomy/enable std_msgs/msg/Empty "{}"
ros2 topic pub --once /tello/autonomy/disable std_msgs/msg/Empty "{}"
ros2 topic pub --once /tello/autonomy/stop std_msgs/msg/Empty "{}"
ros2 topic pub --once /tello/autonomy/land std_msgs/msg/Empty "{}"
ros2 topic pub --once /tello/autonomy/emergency std_msgs/msg/Empty "{}"
```

## Fluxo QR + servo visual

O `qr_node` le `/tello/image_raw`, detecta QR Codes com OpenCV e publica deteccoes em `/vision/qr_codes`. O `visual_servo_node` pode usar `/vision/qr_codes` como entrada por meio do parametro `input_detection_topic`.

O `visual_servo_node` nao le QR diretamente: ele apenas centraliza qualquer alvo visual compativel com o formato de deteccao usado pelo pacote. A missao da Fase 4 ainda nao esta implementada; futuramente essa decisao de missao deve ficar em um `mission_node`.

```text
stream_node
  -> /tello/image_raw
qr_node
  -> /vision/qr_codes
visual_servo_node
  -> /tello/autonomy/cmd_vel
command_mux_node
  -> rc real / takeoff / land
mission_node futuro
  -> decisao de missao
```

Comandos uteis:

```bash
ros2 launch tello_driver qr_servo_test.launch.py show_preview:=true
ros2 launch tello_driver qr_servo_test.launch.py show_preview:=true enable_sdk_init:=true enable_stream_on:=true
ros2 topic echo /vision/qr_codes
ros2 topic echo /vision/qr_debug
ros2 topic echo /tello/autonomy/cmd_vel
```

Esse launch nao inicia `joy_node` nem `command_mux_node`; ele apenas publica o comando calculado em `/tello/autonomy/cmd_vel`.

## Configuracao

Os defaults continuam declarados dentro dos nos. O arquivo `config/tello_default.yaml` existe como referencia opcional para facilitar ajustes futuros.

Para `landing_base_node`, o YAML inclui os limiares HSV de amarelo e azul,
limites de area, morfologia e `publish_mask`. Os defaults sao permissivos para
video ruim do Tello; ajuste primeiro observando `/vision/landing_base_debug` e,
quando necessario, `/vision/landing_base_mask`.

## Proximos passos de autonomia

Ideias planejadas, ainda nao implementadas:

- uso da leitura de QR Code dentro de uma missao;
- missao simplificada inspirada na Fase 4 da Flying Robot League;
- criacao futura de `mission_node`.
