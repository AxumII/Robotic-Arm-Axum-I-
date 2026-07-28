# Robotic Arm Axum I

This repository contains the full design, analysis, embedded code, and experimental validation for the **Axum I**, a 4-DOF articulated robotic arm. The primary axis is driven by a worm gear motor regulated via a closed-loop PI cascade controller implemented on an ESP32 microcontroller.

---

## Table of Contents

1. [General Description & Problem Statement](#1-general-description--problem-statement)
2. [Development Methodology & Flowchart](#2-development-methodology--flowchart)
3. [Engineering Calculations & Design Constraints](#3-engineering-calculations--design-constraints)
   - 3.1 [Torque Analysis & Motor Sizing](#31-torque-analysis--motor-sizing)
   - 3.2 [Kinematic & Dynamic Constraints](#32-kinematic--dynamic-constraints)
   - 3.3 [Structural & Material Resistance](#33-structural--material-resistance)
   - 3.4 [Bill of Materials (BOM) & Component Selection](#34-bill-of-materials-bom--component-selection)
   - 3.5 [CAD Modeling & Assembly Constraints](#35-cad-modeling--assembly-constraints)
4. [MDF Prototype (Proof of Concept)](#4-mdf-prototype-proof-of-concept)
   - 4.1 [MDF CAD Design (v0)](#41-mdf-cad-design-v0)
   - 4.2 [Test Code for Prototype](#42-test-code-for-prototype)
   - 4.3 [Experimental Results & Iteration](#43-experimental-results--iteration)
5. [Mechanical & Physical Design Process](#5-mechanical--physical-design-process)
   - 5.1 [Base Design & Lazy Susan Mechanism](#51-base-design--lazy-susan-mechanism)
   - 5.2 [Shoulder Joint Design](#52-shoulder-joint-design)
   - 5.3 [Elbow Joint Design](#53-elbow-joint-design)
   - 5.4 [Wrist Joint Design](#54-wrist-joint-design)
   - 5.5 [End-Effectors & Tools](#55-end-effectors--tools)
     - 5.5.1 [Calibration Tool](#551-calibration-tool)
     - 5.5.2 [Grippers (v1 & v2)](#552-grippers-v1--v2)
     - 5.5.3 [Electromagnets (v1 & v2)](#553-electromagnets-v1--v2)
   - 5.6 [Control Box Enclosure](#56-control-box-enclosure)
   - 5.7 [Overall Mechanical Assembly](#57-overall-mechanical-assembly)
6. [Engineering Drawings & Schematics](#6-engineering-drawings--schematics)
7. [Assembly Guide](#7-assembly-guide)
   - 7.1 [Base Assembly](#71-base-assembly)
   - 7.2 [Shoulder Assembly](#72-shoulder-assembly)
   - 7.3 [Elbow Assembly](#73-elbow-assembly)
   - 7.4 [Wrist Assembly](#74-wrist-assembly)
   - 7.5 [Tool / End-Effector Integration](#75-tool--end-effector-integration)
8. [Electrical Integration & Wiring](#8-electrical-integration--wiring)
   - 8.1 [Wiring & Connection Diagrams](#81-wiring--connection-diagrams)
   - 8.2 [Soldering & PCB Assembly](#82-soldering--pcb-assembly)
   - 8.3 [Control Box Wiring](#83-control-box-wiring)
9. [Initial Testing & Validation](#9-initial-testing--validation)
   - 9.1 [Unit Code Testing](#91-unit-code-testing)
   - 9.2 [Physical Test Results](#92-physical-test-results)
   - 9.3 [Comparison: Theoretical vs. Practical Results](#93-comparison-theoretical-vs-practical-results)
10. [Motor Control System](#10-motor-control-system)
    - 10.1 [Theoretical Control Models](#101-theoretical-control-models)
    - 10.2 [Firmware Control Loop Implementation](#102-firmware-control-loop-implementation)
    - 10.3 [Final Model Tuning & Selection](#103-final-model-tuning--selection)
11. [Kinematic Modeling](#11-kinematic-modeling)
    - 11.1 [Forward Kinematics (FK)](#111-forward-kinematics-fk)
    - 11.2 [Inverse Kinematics (IK)](#112-inverse-kinematics-ik)
12. [Motion Control & Command Reference](#12-motion-control--command-reference)
    - 12.1 [Joint-Space Configuration (`MovA`)](#121-joint-space-configuration-mova)
    - 12.2 [Unconstrained Point-to-Point Motion (`MovJ`)](#122-unconstrained-point-to-point-motion-movj)
    - 12.3 [Linear Interpolated Motion (`MovL` & `MovLJc`)](#123-linear-interpolated-motion-movl--movljc)
    - 12.4 [Circular Interpolated Motion (`MovC` & `MovCJc`)](#124-circular-interpolated-motion-movc--movcjc)
13. [System Characterization](#13-system-characterization)
    - 13.1 [Technical Datasheet](#131-technical-datasheet)
    - 13.2 [User & Instruction Manual](#132-user--instruction-manual)
14. [Performance Evaluation & Experimental Tests](#14-performance-evaluation--experimental-tests)
    - 14.1 [Angular Positioning Performance](#141-angular-positioning-performance)
    - 14.2 [Free-Space Motion Trajectory](#142-free-space-motion-trajectory)
    - 14.3 [Line-Following Accuracy](#143-line-following-accuracy)
      - 14.3.1 [Joint-Space Control](#1431-joint-space-control)
      - 14.3.2 [Jacobian Differential Control](#1432-jacobian-differential-control)
      - 14.3.3 [Error Analysis & Comparison](#1433-error-analysis--comparison)
    - 14.4 [Circle-Following Accuracy](#144-circle-following-accuracy)
      - 14.4.1 [Joint-Space Control](#1441-joint-space-control)
      - 14.4.2 [Jacobian Differential Control](#1442-jacobian-differential-control)
      - 14.4.3 [Error Analysis & Comparison](#1443-error-analysis--comparison)
    - 14.5 [Pick-and-Place Validation](#145-pick-and-place-validation)
      - 14.5.1 [Horizontal Electromagnet Execution](#1451-horizontal-electromagnet-execution)
      - 14.5.2 [Vertical Electromagnet Execution](#1452-vertical-electromagnet-execution)
15. [Teach Pendant Controller](#15-teach-pendant-controller)
    - 15.1 [Datasheet & Hardware Specifications](#151-datasheet--hardware-specifications)
    - 15.2 [Enclosure CAD Design](#152-enclosure-cad-design)
    - 15.3 [Schematics & Wiring](#153-schematics--wiring)
    - 15.4 [Communication Protocol & Firmware](#154-communication-protocol--firmware)
    - 15.5 [Control Box Integration](#155-control-box-integration)
16. [Teach Pendant Experimental Tests](#16-teach-pendant-experimental-tests)
17. [Conclusions & Future Work](#17-conclusions--future-work)


# 1. General Description & Problem Statement

The **Axum I** is a 4-Degree-of-Freedom (4-DOF) articulated robotic arm developed as a modular platform for trajectory tracking, kinematics evaluation, and embedded control validation. Articulated manipulators face critical challenges regarding structural rigidity, torque distribution across joints, and non-linear dynamic behavior caused by link inertia and joint friction.

---

# 2. Development Methodology & Flowchart

The design and implementation of the Axum I followed an iterative engineering workflow, moving from analytical calculations to physical validation.

---

# 3. Engineering Calculations & Design Constraints

Antes de proceder con el modelado CAD y el ensamblaje físico, se evaluaron modelos analíticos para determinar las restricciones físicas, los requerimientos estructurales y los límites dinámicos del brazo robótico.

## 3.1 Cálculos Generales y Cinemática del Torque

Para asegurar que los actuadores puedan soportar tanto las cargas estáticas de retención como las aceleraciones dinámicas, se evaluaron los requerimientos de torque en cada articulación bajo la condición de máxima exigencia (pose horizontal completamente extendida en voladizo):

$$
\tau_{\text{joint}} = \sum_{i} \left( m_i \cdot g \cdot r_i \cdot \cos(\theta_i) \right) + I_{\text{total}} \cdot \alpha + \tau_{\text{friction}}
$$

Donde:
*   $\tau_{\text{joint}}$: Torque total requerido en el eje de la articulación.
*   $m_i$: Masa del eslabón $i$ y del hardware o carga útil montada.
*   $g$: Aceleración gravitacional (9.81 m/s²).
*   $r_i$: Distancia desde el eje de la articulación hasta el centro de masa del eslabón $i$.
*   $\theta_i$: Orientación angular absoluta relativa al plano horizontal ($\cos(0^\circ) = 1$ para la peor condición de voladizo).
*   $I_{\text{total}}$: Momento de inercia total referido al eje de rotación de la articulación.
*   $\alpha$: Aceleración angular requerida ($\ddot{\theta}$).
*   $\tau_{\text{friction}}$: Pérdidas estimadas por fricción viscosa y de Coulomb en rodamientos y transmisiones.

Para elegir las dimensiones óptimas, se desarrollaron simulaciones espaciales en código. (Nota: Se ignora temporalmente la fricción viscosa $\tau_{\text{friction}}$ en la simulación debido a la dificultad de su medición directa, aunque se asume una tolerancia de seguridad en el diseño final).

**1. Matrices de Transformación Homogénea (Helpers)**
Para calcular las posiciones articulares independientemente de la postura del brazo, el simulador utiliza Transformaciones Homogéneas (4x4). Estas combinan rotación y traslación en una sola operación matricial dependiente del eje de giro:

$$
R_x(\theta) = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & \cos\theta & -\sin\theta & 0 \\ 0 & \sin\theta & \cos\theta & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}, \quad
R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta & 0 \\ 0 & 1 & 0 & 0 \\ -\sin\theta & 0 & \cos\theta & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

Y la matriz de traslación para los desplazamientos (offsets y centros de masa):

$$
\text{Trans}(v_x, v_y, v_z) = \begin{bmatrix} 1 & 0 & 0 & v_x \\ 0 & 1 & 0 & v_y \\ 0 & 0 & 1 & v_z \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

**2. Cinemática Directa (Forward Kinematics)**
Esta función halla la posición exacta en el espacio, su eje de giro (vector Z local) y los centros de masa, multiplicando secuencialmente las matrices desde la base hasta la herramienta (TCP). Cabe destacar que el modelo cinemático establece que la base a hombro es estrictamente vertical y gira de forma puramente axial en el eje Z:

$$
T_i = T_{i-1} \cdot R_{\text{eje}}(\theta_i) \cdot \text{Trans}(\text{offset}_i)
$$

**3. Cálculo de Torques Estáticos Tridimensionales**
El torque estático espacial se calcula mediante el producto cruz ($\times$) entre el vector de distancia radial ($\vec{r}$) y el vector de fuerza gravitacional ($\vec{F}_g$). Para cada articulación $i$, se evalúa el peso de todos los eslabones $j$ posteriores:

$$
\vec{\tau}_{3D} = \sum_{j>i} \left( \vec{r}_j \times (m_j \cdot \vec{g}) \right) + (\vec{r}_{\text{load}} \times (m_{\text{load}} \cdot \vec{g}))
$$

Como los motores solo sufren esfuerzo en su eje de rotación, se aísla esta fuerza usando el producto punto ($\cdot$) con el vector unitario del eje de giro del motor ($\vec{u}_{\text{axis}}$):

$$
\tau_{\text{static}} = \vert{}\vec{\tau}_{3D} \cdot \vec{u}_{\text{axis}}\vert{}
$$

**4. Inercia y Restricciones Dinámicas**
Se calcula la inercia proyectando la distancia perpendicular real ($r_{\perp}$) de las masas al eje de rotación. Con esto, se verifica el remanente de torque dinámico y se despeja la aceleración angular máxima que cada articulación puede soportar sin saturar el motor:

$$
I_{\text{total}} = \sum \left( m_j \cdot r_{\perp, j}^2 \right) \quad \rightarrow \quad \alpha_{\text{max}} = \frac{\tau_{\text{rated}} - \tau_{\text{static}}}{I_{\text{total}}}
$$

## 3.2 Torque Analysis & Motor Sizing

Las restricciones de torque dependen de los actuadores disponibles, exigiendo bajas velocidades angulares para maximizar la relación de engranajes mecánicos. El script de simulación establece límites operativos: 70 kg·cm para el hombro y 21 kg·cm para el resto. En el mercado colombiano actual, las opciones viables se dividen en:

*   **Motores DC Brushed Planetarios:** Torques de 20 a 100 kg·cm (4 a 30 RPM). Costos: 80,000 - 200,000 COP. Incorporan encoders de cuadratura.
*   **Motores DC Brushed Worm:** Torques de 20 a 100 kg·cm (4 a 30 RPM). Costos: 60,000 - 150,000 COP. Ventaja de autobloqueo mecánico por el tornillo sin fin y alta disponibilidad.
*   **Motores AC Brushless:** Torques de 50 a 200 kg·cm. Costos: 80,000 - 300,000 COP. Son más costosos y complejos ya que requieren controladores dedicados, aunque ofrecen el mejor rendimiento.

## 3.3 Kinematic & Dynamic Constraints

Dado el torque analizado, la principal restricción dinámica es la velocidad angular. Las velocidades deben mantenerse bajas; dado que el brazo tiene un alcance inferior a 50 cm, no se exigen velocidades lineales altas. Una mayor velocidad exigiría mayores aceleraciones para frenar entre cambios de pose, amplificando el impacto negativo de una inercia alta.

El entorno virtual compara iterativamente la aceleración objetivo con la calculada ($\alpha_{\text{max}}$). Si la solicitada supera la capacidad remanente del sistema evaluado, el script emite alertas sobre posibles fallos (cuellos de botella).

## 3.4 Structural & Material Resistance

El análisis geométrico evalúa secciones transversales de 30x50 mm para polímeros (PLA y ABS). Estos materiales fueron modelados rigurosamente asumiendo una configuración de fabricación por impresión 3D, aplicando un patrón de relleno cúbico (cubic infill) del 30% para reflejar con precisión su densidad real y la relación peso-resistencia. También se evaluó el MDF como alternativa de bajo coste, asumiendo un perfil macizo mecanizado de 8x40 mm.

El código emplea una función paramétrica donde las longitudes de los eslabones móviles mantienen una proporción estricta respecto a un segmento principal variable ($L$, desde 180 mm hasta 450 mm):
*   **Hombro ($L_2$):** $L$
*   **Codo ($L_3$):** $0.7 \cdot L$
*   **Muñeca ($L_4$):** $0.3 \cdot L$

Además, un modelo de viga en voladizo evalúa la flexión máxima ($\delta$) en el segmento crítico (el hombro), integrando el Módulo de Young efectivo, el peso propio distribuido ($W_{\text{propio}}$) y la carga puntual de las articulaciones distales ($F_{\text{punta}}$):

$$
\delta = \frac{F_{\text{punta}} \cdot L^3}{3 \cdot E \cdot I} + \frac{W_{\text{propio}} \cdot L^3}{8 \cdot E \cdot I}
$$


<img width="1920" height="975" alt="Grafica Torques vs Longitud" src="https://github.com/user-attachments/assets/d853dbce-5ba6-464b-bdf9-768232c26adc" />

<img width="1920" height="975" alt="Deflection vs Width" src="https://github.com/user-attachments/assets/7f761af7-6d26-4d27-a23e-ce8fe1f9fdec" />

<img width="1000" height="800" alt="Cin Pose" src="https://github.com/user-attachments/assets/eb0c463e-0e98-4100-8dfa-58fef866560d" />

A continuación, se presenta la tabla con los resultados para la configuración geométrica elegida ($L = 300\text{ mm}$), evaluando la postura estirada de mayor exigencia, junto con su respectiva representación cinemática tridimensional:

**ANÁLISIS DE POSTURA: [0.0, 0.0, 0.0, 0.0] | Carga: 200.0g | Acel. Deseada: 5.0 rad/s²**

| Articulación | Límite [kg·cm] | T. Estático [kg·cm] | T. Dinámico [kg·cm] | T. Total [kg·cm] | Acel. MAX [rad/s²] | Factor de Seguridad (FS) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **J0 (Z)** | 21.0 | 0.00 | 9.13 | 9.13 | 11.51 | 2.30 |
| **J1 (Y)** | 70.0 | 36.72 | 8.86 | 45.58 | 18.78 | 1.54 |
| **J2 (Y)** | 21.0 | 12.32 | 1.56 | 13.87 | 27.89 | 1.51 |
| **J3 (Y)** | 21.0 | 2.03 | 0.09 | 2.12 | 1080.09 | 9.92 |

> **Resultado de la Simulación:** **ÉXITO**. La aceleración deseada es soportada adecuadamente por los motores seleccionados. (El cuello de botella en todo el brazo se encuentra en 11.51 rad/s²).

Se eligio esa postura dado los factores de seguridad de 1.5, que permiten tener suficiente seguridad sin considerar factores extra como excentricidades, friccion viscosa. Tambien se uso una carga de 200g considerando un caso excesivo, el cual aun es seguro. 

## 3.5 Bill of Materials (BOM) & Component Selection

Con base en los resultados del remanente de torque y la viabilidad del mercado, se establece la siguiente lista de componentes principales requeridos para satisfacer la dinámica del sistema:

*   **Actuadores:** Motorreductores DC tipo Worm equipados con encoders de cuadratura ya integrados para el cierre del lazo de control de posición. Tres iguales y uno de mayor carga para el hombro.
*   **Microcontrolador (Hardware-in-the-Loop):** ESP32-S3 (N16R8) para gestionar las interrupciones de los encoders, la matemática de coma flotante de las matrices cinemáticas y la generación de señales de control PWM.
*   **Mecánica y Transmisión:** Se descarta cualquier tipo de transision extra, complejiza el diseño y no aportan un rendimiento significativo frente al que ya esta, ademas de que encarece el ensamble.

## 3.6 CAD Modeling & Assembly Constraints

El paso de la formulación analítica al software de diseño asistido por computadora (CAD) está condicionado por las siguientes reglas restrictivas derivadas de la simulación:

1.  **Parámetros de Fabricación Aditiva:** Para que la resistencia estructural final corresponda a la matriz analítica de deflexión ($\delta$), las piezas impresas (PLA/ABS) deben parametrizarse en laminadores como PrusaSlicer o Cura respetando el 30% de infill cúbico. Además, se debe aumentar el grosor de las paredes perimetrales (shell thickness) específicamente en las uniones de los motores J1 y J2 para mitigar la fatiga por concentración de esfuerzos.
2.  **Alineación de Ejes:** Como dictamina la matriz de rotación, la articulación de la base debe ser estrictamente vertical. Los planos CAD deben garantizar tolerancias precisas (H7/g6) para los rodamientos de apoyo de la base, previniendo cabeceos durante las aceleraciones altas.
3.  **Distribución de Masas (Inercia):** El diseño debe tener un enfoque que no tenga elementos tan pesados y con la inercia minima posible y de manera mas uniforme para simplificar los calculos.
4.  **Ensamble Practico:** Debe ser diseñado de manera que cada articulacion sea ensamblada y desensamblada de una forma rapida, en caso de falla, cambio de pieza y transporte.
5.  **Uniones con Motores:** Las articulaciones se unen con bridas D Shaft, de modo que se atornillan junto a las articulaciones.
6.  **Tornilleria:** Las articulaciones, la base, las protecciones, las tapas y las herramientas se deben unir con tornilleria M3 y M4, se usan tolerancias de 0.5mm para ajuste y 1mmm para rotacion.


# 4. MDF Prototype (Proof of Concept)

Con el fin de validar el modelo analítico y la cinemática tridimensional antes de incurrir en los costos y tiempos de fabricación de la impresión 3D final, se optó por construir un prototipo de prueba de concepto (PoC) o versión cero (v0) utilizando piezas macizas de MDF. Este enfoque permite realizar iteraciones rápidas de *hardware-in-the-loop* y afinar el control *firmware* de los motores reales bajo carga mecánica.

## 4.1 MDF CAD Design (v0)

El diseño CAD de esta primera iteración hereda estrictamente las dimensiones óptimas dictadas por las simulaciones del apartado anterior. Para los eslabones principales se adoptó un perfil macizo de MDF de 8x40 mm, manteniendo la envergadura base del brazo paramétrico:

*   **Hombro ($L_2$):** 300 mm.
*   **Codo ($L_3$):** 210 mm ($0.7 \cdot L_2$).
*   **Muñeca ($L_4$):** 90 mm ($0.3 \cdot L_2$).

El ensamble fue concebido para ser mecanizado mediante corte láser o ruteadora CNC, incorporando los alojamientos exactos para las bridas *D-Shaft* de los motores DC Brushed (JGY370 y 5840-31ZY). Este modelo CAD preliminar permite evaluar físicamente el espacio de trabajo real, las colisiones propias de la estructura y el enrutamiento del cableado hacia los actuadores y *encoders*.

*(Insertar aquí: Planos de ensamble CAD del prototipo en MDF con cotas generales)*
> **Figura 4.1.1:** Planos generales del prototipo v0 en MDF. Se aprecian las distancias entre centros de rotación (300 mm, 210 mm, 90 mm) y los puntos de anclaje de los motores.

## 4.2 Test Firmware & Control Architecture

Para la validación de los actuadores en el prototipo, se desarrolló un *firmware* de bajo nivel en C++ ejecutado sobre un microcontrolador ESP32-S3 (N16R8). En lugar de utilizar bucles de control avanzados, usa un simple control de velocidad dando porcentajes de PWM y ver la velocidad angular y el angulo relativo en la telemetria. 

**1. Abstracción de Hardware**
El sistema mapea de forma contigua los periféricos (pines de *encoders* y salidas PWM) utilizando arreglos indexados para las cuatro articulaciones. Para solucionar problemas comunes en el ensamblaje mecánico (como motores que quedan físicamente girados respecto a su eje teórico), se implementaron banderas booleanas (`INVERTIR_MOTOR` e `INVERTIR_ENCODER`). Estas variables permiten cruzar por *software* las señales de control de los puentes H (intercambiando los canales PWM izquierdo/derecho) y los canales A/B de los *encoders*. Adicionalmente, el código calcula dinámicamente la resolución total de cada articulación en Pulsos Por Revolución (PPR) multiplicando los pulsos base del *encoder* por 4 (cuadratura) y por la relación de las cajas reductoras mecánicas (1154:1 para el hombro y 1000:1 para el resto).

**2. Gestión de Memoria No Volátil (NVS) Orientada a Eventos**
Para evitar la degradación prematura de la memoria Flash del microcontrolador —causada por escrituras cíclicas continuas en el bucle principal— el sistema registra la posición de manera condicional utilizando la librería `Preferences`. La rutina de escritura en NVS (`guardarPosicionNVS`) captura el conteo absoluto y los *offsets* de calibración de una articulación única y exclusivamente cuando esta alcanza una detención voluntaria (PWM = 0) o cuando se gatilla una parada de emergencia. Este enfoque garantiza que el brazo no pierda su marco de referencia espacial tras un corte súbito de energía, permitiendo reanudar la operación exactamente en la pose donde se detuvo.

**3. Lazo Rápido Anti-Colisión (Seguridad Activa)**
El *firmware* implementa una rutina de seguridad prioritaria que se ejecuta cada 50 milisegundos mediante un temporizador asíncrono. Esta función adquiere la tensión analógica de sensores de corriente a través de un ADC externo de alta precisión (ADS1115) vía I2C. La lectura pasa por un filtro de zona muerta (ignora ruidos menores a 0.03V) y se restan los *offsets* en reposo (`OFFSETS_IS`) antes de aplicar los factores de conversión a amperios específicos para cada familia de motor (20.0 A/V para el 31ZY y 13.6 A/V para los JGY370). Si la corriente resultante supera los límites de bloqueo o *stall current* predefinidos (6.5A para el hombro y 1.2A para las demás), el sistema interrumpe inmediatamente el PWM, bloquea la articulación afectada, registra la colisión y salva el estado en la memoria NVS.

**4. Interfaz de Comandos y Telemetría**
El sistema es gobernado mediante un analizador de comandos por puerto serial (Serial Parser). La arquitectura soporta instrucciones complejas como:
*   **Comandos de Movimiento Temporizado (T):** Permite inyectar un porcentaje de PWM durante una ventana de tiempo estricta en milisegundos y luego aplicar un freno activo automático.
*   **Calibración (Z):** Establece el *offset* dinámico para dictar el "cero" relativo (Home) de una articulación.
paralelo emite tramas de telemetría cada 500 ms reportando el estado del voltaje del banco de baterías, el consumo en amperios y el ángulo absoluto articular (transformado desde el conteo de pulsos descontando el *offset*).

## 4.3 Experimental Results & Iteration

Una vez ensamblado el prototipo en MDF y acoplado el sistema de control basado en el ESP32-S3, se procedió a someter la estructura a pruebas dinámicas y estáticas. El objetivo principal de esta iteración es recopilar datos empíricos que permitan realizar ajustes en el diseño CAD final (v1) orientado a la impresión 3D, evaluando los siguientes aspectos clave:

1.  **Validación de Torques y Consumo de Corriente:**
    *   *(Pendiente: Describir el comportamiento térmico y de consumo eléctrico de los motores JGY370 y 5840-31ZY bajo la carga real. Comparar los picos de corriente obtenidos vía telemetría con los límites teóricos).*
2.  **Precisión Geométrica y Holguras (Backlash):**
    *   *(Pendiente: Registrar las desviaciones observadas en el actuador final (TCP) debido a la flexión del MDF y al juego mecánico natural de las cajas reductoras tipo Worm).*
3.  **Respuesta del Sistema Anti-Colisión y NVS:**
    *   *(Pendiente: Detallar la efectividad de la rutina de seguridad al saturar la carga intencionalmente y confirmar si el restablecimiento de posición desde la memoria Flash funciona correctamente tras un corte de energía).*
4.  **Ajustes Estructurales para la v1 (Impresión 3D):**
    *   *(Pendiente: Listar las conclusiones y modificaciones geométricas requeridas para el modelo final; por ejemplo, reubicación de centros de masa, refuerzo de uniones o ajuste de tolerancias para los acoples D-Shaft).*
