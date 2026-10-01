"""arm_broker — el único nodo que publica en /joint_states.

Andamiaje entregado por el curso. Los bloques IMPLEMENTAR son lo que evalúa el
reto; el resto es instrumentación y se usa tal cual.
"""

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from arm_broker_interfaces.action import MoveArm
from arm_broker_interfaces.msg import QueueState

try:
    from arm_broker import fk
    from arm_broker.politicas import POLITICAS, Pedido
except ImportError:
    from src.arm_broker.arm_broker import fk
    from src.arm_broker.arm_broker.politicas import POLITICAS, Pedido


class ArmBroker(Node):

    def __init__(self):
        super().__init__('arm_broker')

        self.declare_parameter('politica', 'fifo')
        self.declare_parameter('tau_envejecimiento_s', 8.0)
        self.declare_parameter('cola_max', 20)
        self.declare_parameter('paso_max_rad', 1.2)
        self.declare_parameter('duracion_movimiento_s', 3.0)
        self.declare_parameter('pasos_interpolacion', 10)

        nombre = self.get_parameter('politica').value
        if nombre not in POLITICAS:
            raise RuntimeError(f'política desconocida: {nombre}. Hay {list(POLITICAS)}')
        clase = POLITICAS[nombre]
        if nombre == 'prioridad':
            self.politica = clase(self.get_parameter('tau_envejecimiento_s').value)
        else:
            self.politica = clase()

        self.cola_max = int(self.get_parameter('cola_max').value)
        self.paso_max = float(self.get_parameter('paso_max_rad').value)
        self.duracion = float(self.get_parameter('duracion_movimiento_s').value)
        self.pasos = max(1, int(self.get_parameter('pasos_interpolacion').value))

        self.grupo_entrada = ReentrantCallbackGroup()
        self.grupo_worker = MutuallyExclusiveCallbackGroup()

        self.lock = threading.Lock()
        self.pendientes = []
        self.por_goal_id = {}
        self.ejecutando = None
        self.q_actual = [0.0] * 6
        self.n_aceptados = 0
        self.n_rechazados = 0
        self.n_completados = 0
        self._parar = threading.Event()

        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_cola = self.create_publisher(QueueState, '/arm/queue_state', 10)

        self.servidor = ActionServer(
            self,
            MoveArm,
            'move_arm',
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=self.grupo_entrada,
        )

        self.create_timer(0.2, self.publicar_estado_cola,
                          callback_group=self.grupo_worker)

        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()

        self.get_logger().info(
            f'arm_broker listo · política={self.politica.nombre} · '
            f'cola_max={self.cola_max} · único publicador de /joint_states')

    # ========================= IMPLEMENTAR · ítem 2 ==========================
    def goal_callback(self, goal_request):
        """Admisión. Barata e inmediata: acepta o rechaza, nunca ejecuta."""
        q = list(goal_request.joint_positions)

        ok_lim, msg_lim = fk.dentro_de_limites(q)
        if not ok_lim:
            self.get_logger().warn(f'Goal rechazado (límites): {msg_lim}')
            with self.lock:
                self.n_rechazados += 1
            return GoalResponse.REJECT

        ok_ws, msg_ws = fk.dentro_del_workspace(q)
        if not ok_ws:
            self.get_logger().warn(f'Goal rechazado (workspace): {msg_ws}')
            with self.lock:
                self.n_rechazados += 1
            return GoalResponse.REJECT

        with self.lock:
            q_act = list(self.q_actual)
            len_cola = len(self.pendientes)

        paso = fk.paso_articular(q_act, q)
        if paso > self.paso_max:
            self.get_logger().warn(
                f'Goal rechazado (paso {paso:.2f} rad > máximo {self.paso_max:.2f} rad)'
            )
            with self.lock:
                self.n_rechazados += 1
            return GoalResponse.REJECT

        if len_cola >= self.cola_max:
            self.get_logger().warn(f'Goal rechazado (cola llena: {len_cola}/{self.cola_max})')
            with self.lock:
                self.n_rechazados += 1
            return GoalResponse.REJECT

        with self.lock:
            self.n_aceptados += 1

        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """Encolar. AQUÍ NO SE EJECUTA NADA, y no se publica en /joint_states."""
        req = goal_handle.request
        pedido = Pedido(goal_handle, req.client_id, req.priority, req.joint_positions)

        with self.lock:
            self.pendientes.append(pedido)
            self.por_goal_id[pedido.goal_id] = pedido

    def _worker(self):
        """El único que decide a quién le toca. Corre en su propio hilo."""
        while not self._parar.is_set():
            pedido = None
            with self.lock:
                if self.ejecutando is None and self.pendientes:
                    idx = self.politica.siguiente(self.pendientes)
                    if idx is not None and 0 <= idx < len(self.pendientes):
                        pedido = self.pendientes.pop(idx)

            if pedido is None:
                time.sleep(0.05)
                continue

            if pedido.goal_handle.is_cancel_requested:
                pedido.goal_handle.canceled()
                with self.lock:
                    self.por_goal_id.pop(pedido.goal_id, None)
                continue

            with self.lock:
                pedido.t_inicio_ejec = time.time()
                self.ejecutando = pedido

            pedido.goal_handle.execute()

            # Esperar a que termine la ejecución con exclusión mutua
            pedido.fin.wait()

            with self.lock:
                self.ejecutando = None
                self.por_goal_id.pop(pedido.goal_id, None)

            self.politica.atendido(pedido)

    def execute_callback(self, goal_handle):
        """Ejecutar UN pedido. Lo llama el worker, nunca handle_accepted."""
        goal_id = bytes(goal_handle.goal_id.uuid).hex()[:12]
        with self.lock:
            pedido = self.por_goal_id.get(goal_id)

        if not pedido:
            res = MoveArm.Result()
            res.success = False
            res.message = 'Pedido no encontrado'
            goal_handle.abort()
            return res

        t_inicio = time.time()
        wait_time_s = (
            pedido.t_inicio_ejec - pedido.t_llegada
            if pedido.t_inicio_ejec
            else (t_inicio - pedido.t_llegada)
        )

        q_destino = pedido.joint_positions
        with self.lock:
            q_inicio = list(self.q_actual)

        dt = self.duracion / float(self.pasos)
        cancelado = False

        try:
            for step in range(1, self.pasos + 1):
                if goal_handle.is_cancel_requested:
                    cancelado = True
                    break

                frac = step / float(self.pasos)
                q_interp = [q_inicio[i] + frac * (q_destino[i] - q_inicio[i]) for i in range(6)]
                self.mover(q_interp)

                fb = MoveArm.Feedback()
                fb.state = 'EXECUTING'
                fb.queue_position = 0
                fb.elapsed_s = time.time() - t_inicio
                goal_handle.publish_feedback(fb)

                time.sleep(dt)

            exec_time_s = time.time() - t_inicio
            res = MoveArm.Result()
            res.wait_time_s = float(wait_time_s)
            res.exec_time_s = float(exec_time_s)

            if cancelado or goal_handle.is_cancel_requested:
                res.success = False
                res.message = 'Movimiento cancelado'
                goal_handle.canceled()
            else:
                res.success = True
                res.message = 'Movimiento completado con éxito'
                with self.lock:
                    self.n_completados += 1
                goal_handle.succeed()

            return res

        finally:
            pedido.fin.set()
    # =========================================================================

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    # ----------------------------------------------------------- publicar
    def mover(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = fk.JOINT_NAMES
        msg.position = [float(v) for v in q]
        self.pub_joint.publish(msg)
        with self.lock:
            self.q_actual = list(q)

    def publicar_estado_cola(self):
        msg = QueueState()
        msg.stamp = self.get_clock().now().to_msg()
        with self.lock:
            ej = self.ejecutando
            msg.executing_client = ej.client_id if ej else ''
            msg.executing_goal_id = ej.goal_id if ej else ''
            msg.executing_elapsed_s = (time.time() - ej.t_inicio_ejec) if ej and ej.t_inicio_ejec else 0.0
            msg.queue_length = len(self.pendientes)
            msg.queued_goal_ids = [p.goal_id for p in self.pendientes]
            msg.queued_clients = [p.client_id for p in self.pendientes]
            msg.queued_priorities = [min(255, max(0, p.priority)) for p in self.pendientes]
            msg.queued_wait_s = [p.espera_s for p in self.pendientes]
            msg.total_accepted = self.n_aceptados
            msg.total_rejected = self.n_rechazados
            msg.total_completed = self.n_completados
            cola = list(self.pendientes)
        self.pub_cola.publish(msg)

        for posicion, p in enumerate(cola, start=1):
            try:
                fb = MoveArm.Feedback()
                fb.state = 'QUEUED'
                fb.queue_position = posicion
                fb.elapsed_s = p.espera_s
                p.goal_handle.publish_feedback(fb)
            except Exception:
                pass

    def destroy_node(self):
        self._parar.set()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = ArmBroker()
    executor = MultiThreadedExecutor()
    executor.add_node(nodo)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
