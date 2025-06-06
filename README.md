# 🐢 Taller ROS 2: Comunicación entre Nodos con Turtlesim

## 🎯 Objetivo del taller

Desarrollar dos nodos ROS 2 que se comuniquen entre sí mediante tópicos para controlar el movimiento de *Turtlesim* en un patrón **triangular**, utilizando la información de odometría.

---

## 🧱 Parte 1: Crear el Workspace

Antes de comenzar a desarrollar con ROS 2, es necesario crear un **workspace**, que es un entorno de trabajo donde se almacenarán y compilarán todos los paquetes del proyecto.

Los siguientes comandos te permiten crear y preparar ese entorno:

```bash
mkdir -p ~/ros2_ws/src
```

Este comando crea una carpeta llamada `ros2_ws` dentro de tu directorio personal (`~`) y, dentro de ella, una subcarpeta llamada `src`. Esta carpeta `src` es esencial, ya que es donde se colocarán los paquetes ROS que vas a crear o clonar. `colcon build` buscará automáticamente los paquetes dentro de esta carpeta para compilarlos.

```bash
cd ~/ros2_ws
colcon build
```

Después de crear la estructura del workspace, cambiamos al directorio raíz (`ros2_ws`) y ejecutamos `colcon build`, que es la herramienta recomendada en ROS 2 para compilar workspaces. Esta orden compila todos los paquetes dentro de `src` y genera las carpetas `build/`, `install/` y `log/`. La carpeta `install/` es la que contendrá los archivos necesarios para ejecutar los nodos y scripts.

```bash
source install/setup.bash
```

Este comando configura el entorno actual de la terminal para que reconozca los paquetes y nodos que has compilado en tu workspace. Es **muy importante** ejecutar este comando **en cada nueva terminal** antes de correr cualquier nodo.


## 📦 Parte 2: Crear el paquete

Creamos un paquete llamado `turtle_controller` con soporte para Python y las dependencias necesarias:

```bash
cd ~/ros2_ws/src
ros2 pkg create turtle_controller --build-type ament_python --dependencies rclpy geometry_msgs turtlesim
```

Estructura esperada del paquete:

```
turtle_controller/
├── turtle_controller/
│   └── __init__.py
├── package.xml
├── setup.cfg
├── setup.py
```

---

## 🐢 Parte 3: Ejecutar Turtlesim

En una nueva terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

---

## 📥 Parte 4: Nodo Subscriber — `odom_listener.py`

Creamos el archivo:

```bash
cd ~/ros2_ws/src/turtle_controller/turtle_controller
touch odom_listener.py
chmod +x odom_listener.py
```

### 📘 ¿Qué hace este nodo?

* Se suscribe al tópico `/turtle1/pose`, donde Turtlesim publica su odometría.
* Calcula la distancia total recorrida por la tortuga sumando los desplazamientos entre cada par de puntos.
* Publica esa distancia acumulada en el tópico `/turtle1/path_length`.
* Muestra la distancia total en consola y permite visualizarla con `rqt_plot`.

### 📄 Código:

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32
import math

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(Float32, '/turtle1/path_length', 10)

        self.last_pose = None
        self.total_distance = 0.0

    def pose_callback(self, msg):
        if self.last_pose is not None:
            dx = msg.x - self.last_pose.x
            dy = msg.y - self.last_pose.y
            increment = math.hypot(dx, dy)
            self.total_distance += increment

        self.last_pose = msg

        msg_out = Float32()
        msg_out.data = self.total_distance
        self.publisher.publish(msg_out)

        self.get_logger().info(f"Distancia total recorrida: {self.total_distance:.2f} unidades")

def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 📄 Código explicado:

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32
import math
```
* Se importan los módulos necesarios:

  * `Pose`: para recibir la posición de la tortuga.
  * `Float32`: para publicar la distancia total recorrida.
  * `math`: para usar funciones como `hypot` que calcula distancia euclidiana.

```python
class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
```
* Se define un nodo llamado `odom_listener`.

```python
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(Float32, '/turtle1/path_length', 10)
```
* El nodo:

  * Se suscribe al tópico `/turtle1/pose`.
  * Publica en `/turtle1/path_length` la distancia total recorrida acumulada.

```python
        self.last_pose = None
        self.total_distance = 0.0
```
* Guarda la última posición conocida para poder calcular los incrementos de distancia.

```python
    def pose_callback(self, msg):
        if self.last_pose is not None:
            dx = msg.x - self.last_pose.x
            dy = msg.y - self.last_pose.y
            increment = math.hypot(dx, dy)
            self.total_distance += increment

        self.last_pose = msg
```
* Cada vez que recibe una nueva pose:

  * Calcula la distancia recorrida desde la última.
  * La suma al total acumulado.
  * Actualiza la última posición.

```python
        msg_out = Float32()
        msg_out.data = self.total_distance
        self.publisher.publish(msg_out)
```
* Publica el total acumulado como un `Float32`.

```python
        self.get_logger().info(f"Distancia total recorrida: {self.total_distance:.2f} unidades")
```
* Imprime por consola la distancia total con dos decimales.

```python
def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

* Estructura estándar de ROS 2 para inicializar, ejecutar y finalizar el nodo.
---

## 📤 Parte 5: Nodo Publisher — `triangle_stepper.py`

Primero, creamos el archivo del nodo:

```bash
touch triangle_stepper.py
chmod +x triangle_stepper.py
```

### 📘 ¿Qué hace este nodo?

Este nodo controla el movimiento de la tortuga para que recorra un **triángulo equilátero** de manera precisa, utilizando la información de odometría publicada en `/turtle1/pose`.

El comportamiento es el siguiente:

* La tortuga **calcula tres vértices** que forman un triángulo equilátero, usando su posición de inicio como primer punto.
* Se **gira en el lugar** hasta alinearse con el siguiente vértice.
* Luego **se mueve recto** exactamente 3 unidades hasta ese vértice.
* Se detiene brevemente, **gira hacia el siguiente vértice**, y repite.
* El triángulo se repite **indefinidamente** con precisión geométrica.


### 📄 Código:

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class TriangleStepper(Node):
    def __init__(self):
        super().__init__('triangle_stepper')
        self.pose = None
        self.state = 'turn'
        self.origin = None
        self.target = None
        self.angle_target = None
        self.pause_time = None

        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.L = 3.0
        self.epsilon = 0.01
        self.vertices = []
        self.vertex_index = 0

    def update_pose(self, msg):
        self.pose = msg

    def control_loop(self):
        if not self.pose:
            return

        msg = Twist()

        # Inicializar triángulo solo una vez
        if not self.vertices:
            x0, y0 = self.pose.x, self.pose.y
            p1 = (x0, y0)
            p2 = (x0 + self.L, y0)
            p3 = (x0 + self.L / 2, y0 + (self.L * math.sqrt(3)) / 2)
            self.vertices = [p1, p2, p3]
            self.origin = p1
            self.target = p2
            self.vertex_index = 1
            self.state = 'turn'
            return

        dx = self.target[0] - self.pose.x
        dy = self.target[1] - self.pose.y
        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - self.pose.theta)

        if self.state == 'turn':
            if abs(angle_diff) > self.epsilon:
                msg.angular.z = 1.0 * angle_diff
            else:
                msg.angular.z = 0.0
                self.state = 'pause'
                self.pause_time = self.get_clock().now()
                self.origin = (self.pose.x, self.pose.y)

        elif self.state == 'pause':
            now = self.get_clock().now()
            if (now - self.pause_time).nanoseconds / 1e9 > 0.3:
                self.state = 'move'

        elif self.state == 'move':
            traveled = math.hypot(self.pose.x - self.origin[0], self.pose.y - self.origin[1])
            if traveled < self.L:
                msg.linear.x = 1.5
            else:
                msg.linear.x = 0.0
                self.state = 'turn'
                self.vertex_index = (self.vertex_index + 1) % 3
                self.target = self.vertices[self.vertex_index]

        self.publisher.publish(msg)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = TriangleStepper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 📄 Código explicado:

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
```
* Se importan los módulos necesarios para crear un nodo, suscribirse a odometría y publicar comandos de movimiento.

```python
class TriangleStepper(Node):
    def __init__(self):
        ...
```
* Se define el nodo `triangle_stepper` que controla el movimiento a través de odometría y lógica de trayectoria.

```python
        self.vertices = []
        self.vertex_index = 0
```

* Se define una lista con los 3 puntos del triángulo que se recorrerán en bucle.

```python
    def control_loop(self):
        ...
```
* Esta función se ejecuta cada 0.1 segundos y define el comportamiento secuencial:

  * **`turn`**: girar hacia el siguiente vértice.
  * **`pause`**: detenerse brevemente para asegurar estabilidad.
  * **`move`**: avanzar en línea recta hasta alcanzar el siguiente vértice.

```python
        if not self.vertices:
            ...
```
* Se inicializan los tres vértices del triángulo en función de la posición actual de la tortuga.

```python
        self.angle_target = self.normalize_angle(...)
```

* Se calcula el ángulo que la tortuga debe alcanzar para estar alineada con el siguiente vértice.

```python
        self.publisher.publish(msg)
```

* Se publican los comandos de velocidad en `/turtle1/cmd_vel` para mover la tortuga.

---

## ⚙️ Parte 6: Configuración del archivo `setup.py`

Para que los nodos Python del paquete puedan ser ejecutados directamente con `ros2 run`, es necesario registrarlos en el archivo `setup.py` dentro de la sección `entry_points`.

Si ya tienes este archivo generado automáticamente, modifícalo así:

### 📝 Modificación del archivo `setup.py`

```python
from setuptools import find_packages, setup

package_name = 'turtle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anthony',
    maintainer_email='anthony@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_listener = turtle_controller.odom_listener:main',
            'triangle_mover = turtle_controller.triangle_mover:main',
        ],
    },
)
```

### 🔍 Explicación

* En `console_scripts`, se define una lista de scripts que ROS 2 podrá ejecutar como nodos.
* Cada entrada tiene el formato:

  ```bash
  nombre_comando = paquete.nombre_archivo:función_principal
  ```

  Por ejemplo:

  * `odom_listener` es el nombre con el que ejecutarás el nodo:

    ```bash
    ros2 run turtle_controller odom_listener
    ```
  * Este nombre se vincula con el archivo `odom_listener.py` dentro de la carpeta del paquete y su función `main()`.

⚠️ **Nota importante:** Asegúrate de que ambos archivos (`odom_listener.py` y `triangle_mover.py`) estén dentro de la carpeta `turtle_controller/turtle_controller/`, y que tengan permisos de ejecución. Posicionarse en la carpeta que contiene a los nodos (turtle_controller) y ejecutar:

```bash
chmod +x odom_listener.py
chmod +x triangle_mover.py
```

---

## 🚀 Parte 7: Compilar y ejecutar

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Lanzar los nodos en terminales separadas:

```bash
ros2 run turtle_controller odom_listener
ros2 run turtle_controller triangle_mover
```
---

## 📤 Entregables

Cada estudiante debe entregar:

- ✅ El archivo `odom_listener.py`.
- ✅ El archivo `triangle_mover.py`.
- ✅ Un video donde se vea la tortuga moviéndose en un patrón triangular, mientras ambos nodos están activos.
