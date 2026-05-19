# Duckiebot SLAM DTProject

Dit project is een Duckietown-compatibel ROS project dat SLAM-functionaliteit implementeert voor een Duckiebot. Het is gebaseerd op de Duckietown Developer Manual en de bijgevoegde SLAM-bestanden.

## Projectstructuur

- `packages/duckiebot_slam/`: Het Catkin package met de ROS nodes.
  - `src/slam_node.py`: De hoofd-SLAM node die odometrie en visuele kenmerken combineert.
  - `src/camera_reader_node.py`: Node voor het bekijken van de camerabeelden.
  - `src/wheel_encoder_reader_node.py`: Node voor het uitlezen van wielencoders.
  - `src/wheel_control_node.py`: Node voor directe wielaansturing.
  - `src/twist_control_node.py`: Node voor chassis-level (v, omega) aansturing.
- `launchers/`: Scripts om de verschillende nodes te starten.
- `Dockerfile`: Configuratie voor het bouwen van de Docker image.

## Gebruik

### Bouwen
Bouw het project lokaal met:
```bash
dts devel build -f
```

### Uitvoeren
Voer de hoofd-SLAM node uit op een robot:
```bash
dts devel run -H <ROBOT_NAME>
```

Of gebruik een specifieke launcher (bijv. camera reader) lokaal verbonden met de robot:
```bash
dts devel run -R <ROBOT_NAME> -L camera-reader -X
```

## Componenten uit de handleiding
Dit project bevat implementaties voor:
1. **New ROS DTProject**: Volledige structuur met Dockerfile.
2. **Catkin Packages**: `duckiebot_slam` package met `package.xml` en `CMakeLists.txt`.
3. **ROS Publisher/Subscriber**: Gebruik van de `DTROS` klasse.
4. **Subscribe to Camera**: `camera_reader_node.py`.
5. **Subscribe to Wheel Encoders**: `wheel_encoder_reader_node.py`.
6. **Publish to Wheels**: `wheel_control_node.py`.
7. **Publish Chassis-level Commands**: `twist_control_node.py`.
