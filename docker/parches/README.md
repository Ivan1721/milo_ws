# Parches a drivers externos

El Dockerfile los aplica con `git apply` justo después de descargar el driver, antes de compilarlo.

## `orbbec_camera_v1.2.9.patch` (driver de la Gemini Plus, `docker/camara.repos`)

Corrige fallas medidas el 30-10-2026 con la cámara real (firmware RD3013). Solo toca
`orbbec_camera/src/ob_camera_node.cpp`:

| Falla sin el parche | Efecto | Con el parche |
|---|---|---|
| El driver pide la autoexposición del IR, que la Gemini no tiene (`Unsupported property! id=2025`) | Salta los ajustes del filtro de profundidad que vienen después | Solo la pide si la cámara la soporta |
| `camera_info` sale sin `frame_id` ni `stamp` | Otro nodo no puede ubicar la imagen en el árbol de TF | Lleva el frame y la hora de su imagen |
| Con `depth_registration:=true`, las imágenes salen con el frame `color_optical_frame` (nombre sin `camera_name`) y la nube con el de profundidad | Ese frame no existe en el URDF y la nube queda mal ubicada | Profundidad y nube salen en `<camera_name>_color_optical_frame`, y IR en su propio frame |

Sigue sin arreglar: con `depth_registration:=true`, `ir/camera_info` dice 640×480 y trae los
intrínsecos de color, aunque la imagen IR sea 640×400. En ese modo no hay que usar esos intrínsecos.
