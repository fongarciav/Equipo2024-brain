# Modo MEMORY en el detector de carril

## ¿Qué es la "memoria"?

El detector guarda la **última geometría válida del carril** en dos variables:

- `prev_left_fit`: coeficientes del polinomio de la línea izquierda
- `prev_right_fit`: coeficientes del polinomio de la línea derecha

Esas variables se **actualizan** cada vez que hay una detección válida (STEREO, MONO_RIGHT o MONO_LEFT). No se borran cuando falla la detección.

---

## ¿Cuándo se activa el modo MEMORY?

El detector usa una **cascada de 4 niveles**. En cada frame se prueba en orden:

| Nivel | Modo        | Condición |
|-------|-------------|-----------|
| 1     | **STEREO**  | Se detectan **ambas** líneas (izq y der) y son válidas (distancia mínima, no se cruzan). |
| 2     | **MONO_RIGHT** | No hay STEREO pero **sí** hay línea derecha → se reconstruye la izquierda. |
| 3     | **MONO_LEFT**  | No hay STEREO ni MONO_RIGHT pero **sí** hay línea izquierda → se reconstruye la derecha. |
| 4     | **MEMORY**  | **Ninguno** de los anteriores funcionó (`detection_mode` sigue en `"NONE"`) **y** existe memoria (`prev_left_fit` y `prev_right_fit` no son `None`). |

Es decir: **MEMORY se activa solo cuando en el frame actual no se pudo usar ni STEREO, ni MONO_RIGHT, ni MONO_LEFT**, y además en algún frame anterior sí se guardó un carril válido.

---

## ¿Qué hace cuando está en MEMORY?

- **No** usa ninguna detección del frame actual.
- Usa **solo** `prev_left_fit` y `prev_right_fit` como si fueran las líneas del carril.
- Con esos polinomios se calcula:
  - centro del carril
  - ángulo de desviación
  - ángulo de curvatura
- Ese ángulo se envía al PID y el coche **gira según la geometría antigua** (la del último frame donde sí se detectó bien).
- En la vista de debug las líneas se dibujan en **amarillo** y aparece el texto `MODE: MEMORY`.

---

## Efecto práctico

- **Ventaja:** Si la detección falla un frame (sombra, reflejo, curva fuerte), el coche no se queda sin referencia: sigue usando la última buena.
- **Desventaja:** Si la pista **cambió** (por ejemplo entras en una curva), la memoria sigue siendo la de **antes** (recta o otra curva). El ángulo que se manda puede ser **incorrecto** durante varios frames hasta que vuelva a haber detección real. Por eso en curvas suele “fallar” cuando se pasa mucho tiempo en MEMORY.

---

## Cómo desactivar la memoria desde el dashboard

En la sección **Auto-Pilot** hay un checkbox:

- **"Usar memoria (fallback)"**  
  - **Marcado:** Comportamiento actual (si falla todo, se usa MEMORY).  
  - **Desmarcado:** Si fallan STEREO, MONO_RIGHT y MONO_LEFT, **no** se usa memoria; se devuelve “sin carril” y el autopilot no manda ángulo ese frame (solo detección en vivo).

Así puedes probar **solo con la detección que existe** en cada frame, sin fallback a la memoria.
