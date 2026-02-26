# Práctica 3: Avance y giro del Kobuki

En esta práctica debes crear un nodo de ROS 2 que, aplicando un FSM, haga que el robot ejecute dos movimientos de forma consecutiva:
1. Avanzar L metros
2. Girar N radianes
De tal forma que el movimiento final que describa sea el de un triángulo rectángulo

Después de dibujar el triángulo, el robot debe parar.
.

Debes usar TFs (en particular odom->base_footprint) para determinar cuando el robot ha girado o avanzado
suficiente.
