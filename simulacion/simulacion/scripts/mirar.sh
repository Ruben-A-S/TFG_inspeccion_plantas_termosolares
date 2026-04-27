#!/bin/bash
# Comprobamos si nos has dado un número
if [ -z "$1" ]; then
    echo "❌ ¡Error! Tienes que decirme los grados."
    echo "💡 Ejemplo: ./mirar.sh 45"
    exit 1
fi

echo "🔄 Girando cámara a $1 grados..."

# Hacemos la matemática de grados a radianes directamente en bash
RADIANES=$(awk "BEGIN {print $1 * (3.14159265 / 180.0)}")

# Enviamos el comando a ROS 2
ros2 topic pub --once /parametros_control std_msgs/msg/Float64MultiArray "{data: [$RADIANES, 1.5, 0.0]}" > /dev/null 2>&1

echo "✅ ¡Listo!"
