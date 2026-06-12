# Parámetros del Gemelo Digital (config)

El archivo `qav250_params.yaml` contiene todas las constantes dependientes del chasis y la aerodinámica que alimentan a los diferentes nodos y al modelo Euler-Lagrange.

## Diccionario de Parámetros Clave

- `m`: Masa consolidada del dron, incluyendo hélices, chasis, Pixhawk, PDB y la masa inercial de la porción móvil del stand de pruebas.
- `Ixx`, `Iyy`, `Izz`: Tensores de inercia extraídos de software CAD (SolidWorks) sumados al componente del eje respectivo del banco de pruebas FFT.
- `k` (Coeficiente de Empuje Ascendente): Escalar que define la proporción directa entre la velocidad rotacional angular de la hélice al cuadrado y el vector de fuerza de sustentación en $Z$.
- `b` (Coeficiente de Resistencia Perfil): Constante de fricción parasitaria rotacional para el modelado en guiñada (Yaw).
- `Ax`, `Ay`, `Az`: Coeficientes de arrastre del aire. Previene derivas sin límite en simulación cuando el dron carece de controles cerrados de posición, emulando la fricción de la masa de aire.
- `bloquear_xy`: Condicional lógico para el stand de pruebas FFT. Al activarse, detiene la integración cinemática en X y Y para modelar únicamente un drone capturado en un eje vertical y rotacional, tal cual sucede físicamente en pruebas de estátor.
