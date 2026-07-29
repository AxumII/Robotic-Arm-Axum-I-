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

PONER DATASHEET DE MOTORES ORDENADOS 
adasdasdsadadsa


Se eligieron los modelos de JGY370 de 6RPM y del 31ZY de 7RPM. 

<img width="697" height="593" alt="5840 31yz wire" src="https://github.com/user-attachments/assets/acdd99d2-4ece-471c-b8b9-12ddcca6681b" />
<img width="550" height="769" alt="5840 31yz tabla" src="https://github.com/user-attachments/assets/c2f40718-941a-4ac8-85d1-78acd427fc5a" />
<img width="730" height="674" alt="5840 31yz dim2" src="https://github.com/user-attachments/assets/c78d3d6b-aca1-471c-b3bf-442286a93900" />

<img width="796" height="361" alt="JGY370 dim2" src="https://github.com/user-attachments/assets/341f0b12-39a3-4bde-8757-3efc8a8b93c0" />
<img width="809" height="497" alt="JGY370 dim" src="https://github.com/user-attachments/assets/7e1b6160-4384-4e8b-9d1a-2d93487067bb" />
<img width="726" height="624" alt="JGY370 tabla" src="https://github.com/user-attachments/assets/1ff6cc2f-81fb-4006-9c11-57d8bdc4a98c" />



## 3.6 CAD Modeling & Assembly Constraints

El paso de la formulación analítica al software de diseño asistido por computadora (CAD) está condicionado por las siguientes reglas restrictivas derivadas de la simulación:

1.  **Parámetros de Fabricación Aditiva:** Para que la resistencia estructural final corresponda a la matriz analítica de deflexión ($\delta$), las piezas impresas (PLA/ABS) deben parametrizarse en laminadores como PrusaSlicer o Cura respetando el 30% de infill cúbico. Además, se debe aumentar el grosor de las paredes perimetrales (shell thickness) específicamente en las uniones de los motores J1 y J2 para mitigar la fatiga por concentración de esfuerzos.
2.  **Alineación de Ejes:** Como dictamina la matriz de rotación, la articulación de la base debe ser estrictamente vertical. Los planos CAD deben garantizar tolerancias precisas (H7/g6) para los rodamientos de apoyo de la base, previniendo cabeceos durante las aceleraciones altas.
3.  **Distribución de Masas (Inercia):** El diseño debe tener un enfoque que no tenga elementos tan pesados y con la inercia minima posible y de manera mas uniforme para simplificar los calculos.
4.  **Ensamble Practico:** Debe ser diseñado de manera que cada articulacion sea ensamblada y desensamblada de una forma rapida, en caso de falla, cambio de pieza y transporte.
5.  **Uniones con Motores:** Las articulaciones se unen con bridas D Shaft, de modo que se atornillan junto a las articulaciones.
6.  **Tornilleria:** Las articulaciones, la base, las protecciones, las tapas y las herramientas se deben unir con tornilleria M3 y M4, se usan tolerancias de 0.5mm para ajuste y 1mmm para rotacion.
7.  **Acople de Bridas:**Se debe diseñar ranuras especificas que permitan en ciertas posiciones apretar y aflojar las bridas para el desarme, lo cual puede debilitar la estructura. 


# 4. MDF Prototype (Proof of Concept)

Con el fin de validar el modelo analítico y la cinemática tridimensional antes de incurrir en los costos y tiempos de fabricación de la impresión 3D final, se optó por construir un prototipo de prueba de concepto (PoC) o versión cero (v0) utilizando piezas macizas de MDF. Este enfoque permite realizar iteraciones rápidas de *hardware-in-the-loop* y afinar el control *firmware* de los motores reales bajo carga mecánica.

## 4.1 MDF CAD Design (v0)

El diseño CAD de esta primera iteración hereda estrictamente las dimensiones óptimas dictadas por las simulaciones del apartado anterior. Para los eslabones principales se adoptó un perfil macizo de MDF de 8x40 mm, manteniendo la envergadura base del brazo paramétrico:

*   **Hombro ($L_2$):** 250 mm.
*   **Codo ($L_3$):** 200 mm ($0.7 \cdot L_2$).
*   **Muñeca ($L_4$):** 90 mm ($0.3 \cdot L_2$).

El ensamble fue concebido para ser mecanizado mediante corte láser y taladro casero, incorporando los alojamientos exactos para las bridas *D-Shaft* de los motores DC Brushed (JGY370 y 5840-31ZY). Este modelo CAD preliminar permite evaluar físicamente el espacio de trabajo real, las colisiones propias de la estructura y el enrutamiento del cableado hacia los actuadores y *encoders*.

*(Insertar aquí: Planos de ensamble CAD del prototipo en MDF con cotas generales)*
> **Figura 4.1.1:** Planos generales del prototipo v0 en MDF. Se aprecian las distancias entre centros de rotación (300 mm, 210 mm, 90 mm) y los puntos de anclaje de los motores.

NO LOS ENCUENTRO XDXDXD
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

1.  **Validación de Torques, Velocidades y Consumo de Corriente:**
    *   Al usar cargas de prueba de 200g, los motores mostraron un consumo mayor que sin carga, sin embargo, nada distinto a lo mostrado por los datasheet
    *   De igual manera con las velocidades angulares, siendo solo ligeramiente inferiores a 4RPM cuando tenia una carga, correspondiendose al datasheet de ambos motores. 
2.  **Precisión Geométrica y Holguras (Backlash):**
    *   *(Pendiente: Registrar las desviaciones observadas en el actuador final (TCP) debido a la flexión del MDF y al juego mecánico natural de las cajas reductoras tipo Worm).*
    *   Para este apartado se evidencio que si bien, parece tener cierta presicion en marcar angulos, el juego mecanico de los motores es de entre 2 a 3°, ademas de añadir el juego mecanico cuando no se aprieta correctamente el prisionero de la brida D shaft.
3.  **Respuesta del Sistema Anti-Colisión y NVS:**
    *   Se guarda correctamente la posicion registrada cuando se desenergiza, optimo para no depender de ajustes a un home antes de desenergizar.
4.  **Ajustes Estructurales para la v1 (Impresión 3D):**
    *   *(Pendiente: Listar las conclusiones y modificaciones geométricas requeridas para el modelo final; por ejemplo, reubicación de centros de masa, refuerzo de uniones o ajuste de tolerancias para los acoples D-Shaft).*
    *   Es necesario corregir el soporte axial, ya que este no soporta una carga axial y tiende a tanto deflectar la base donde se apoya el motor impidiendo el giro en el eje Z.
    *   Los motores soportan perfectamente las cargas radiales y los torques propuestos, aunque si es menester reforzar la estructura para evitar deflexiones.

<img width="1599" height="899" alt="image" src="https://github.com/user-attachments/assets/92089393-9302-403e-8fcd-d3eca551c33d" />
<img width="1599" height="899" alt="image" src="https://github.com/user-attachments/assets/63f6986d-2d6a-4d18-8f7b-44ac6d6fac5d" />
<img width="1599" height="899" alt="image" src="https://github.com/user-attachments/assets/479d42b9-4247-4aa9-b7c7-b79fd48ef031" />

TOCA BUSCAR EL VIDEO DE ESTA MONDA MOVIENDOSE :V 


# 5. Mechanical & Physical Design Process

Tras la validación analítica y experimental de la prueba de concepto en MDF, se procedió a la fase de diseño CAD detallado y diseño para fabricación aditiva (DFAM - *Design for Additive Manufacturing*). En esta sección se documenta el diseño tridimensional de las piezas finales impresas en PLA, optimizadas para soportar las cargas calculadas, alojar la electrónica y garantizar un ensamblaje preciso con tornilleria.

## 5.1 Base Design & Lazy Susan Mechanism

El diseño de la base (Articulación J0) inicialmente se realizo atornillando el motor directamente en una base impresa en 3D y la brida soportando todo el peso del brazo, dado a que esta acoplado a una caja que soporta el motor del hombro 31ZY. Si bien, el motor JGY370 de la base funciona, se deflecta lo suficiente la brida de modo que funciona de una manera ineficiente con el TCP alejado del origen. 

<img width="985" height="736" alt="image" src="https://github.com/user-attachments/assets/721f7b1e-2eb6-4619-8aae-656a9971c3af" />

AÑADIR LA IMAGEN REAL DE ESTA VAINA

Dado a que el problema es que el elemento que se encargaba de la rotacion tambien se encargaba de la carga, la solucion consistia en crear un soporte de modo que soporte la carga axial y evite deformaciones para que el motor solo se encargue de la rotacion.
Se evaluo soluciones con rodamientos axiales y radiales, ademas de un cardan de 8mm, sin embargo, la solucion mas practica consiste en adaptar un soporte Lazy Susan, que se encarga de soportar la carga axial en su totalidad como permitir rotacion dado a el sistema interno de rodamientos.

<img width="204" height="192" alt="image" src="https://github.com/user-attachments/assets/66306189-500a-4f0b-b816-a1e7da78a6a0" />

De modo que al implementar el modelo se opto por anclar el soporte del motor 5840 31ZY al Lazy Susan y la otra cara de este ultimo en una base nueva impresa adaptada. 

<img width="613" height="538" alt="image" src="https://github.com/user-attachments/assets/946788b8-c13e-463d-8ee8-e7fda6c9364e" />
<img width="817" height="728" alt="image" src="https://github.com/user-attachments/assets/5707a175-f14d-4135-9244-c9fea8054fbb" />


PONER IMAGENES EXTRA



PONER MAS CARRETA

## 5.2 Shoulder Joint Design

El hombro (J1) es la articulación sometida al mayor torque estático y dinámico. Su geometría fue engrosada y reforzada con un mayor número de paredes perimetrales (*shell thickness*) en el laminador. El diseño incluye el alojamiento específico para el motorreductor 5840-31ZY y una brida de transmisión directa (*D-Shaft*) que previene el deslizamiento bajo cargas pesadas.
*   *(Pendiente: Añadir vistas en sección del anclaje del motor y el enrutamiento de cables para J2 y J3).*

*   EXPLICACION DE ESTO 
PONER MAS CARRETA
## 5.3 Elbow Joint Design

El segmento del codo (J2) fue diseñado con un enfoque de reducción de masa sin comprometer la rigidez estructural. Se utilizó topología aligerada donde el análisis de deflexión lo permitió. Aloja un motor JGY370 encargado de levantar la masa remanente de la muñeca y la herramienta.
*   *(Pendiente: Mostrar capturas del modelo CAD resaltando las zonas aligeradas y los puntos de inserción de tornillería M3/M4).*

*   EXPLICACION DE ESTO
*   PONER MAS CARRETA

## 5.4 Wrist Joint Design

La muñeca (J3) alberga el actuador final responsable del cabeceo de la herramienta. Dado que se ubica en el extremo distal, su peso fue minimizado al máximo. Cuenta con una interfaz de montaje estandarizada (brida universal) que permite el intercambio rápido de distintos efectores finales (*End-Effectors*).
*   *(Pendiente: Ilustrar el mecanismo de acople rápido y los límites físicos de rotación).*
*   PONER MAS CARRETA

## 5.5 End-Effectors & Tools

El brazo fue concebido como una plataforma modular capaz de interactuar con diversos objetos mediante efectores intercambiables que se conectan a la muñeca, con sistema de brida de agujeros M3 a 16mm (Las bridas D Shaft de los JGY370), los cuales se describen a continuación:

### 5.5.1 Calibration Tool
Una herramienta rígida de geometría puntiaguda utilizada exclusivamente para las rutinas de *Homing* y calibración espacial. Permite al usuario alinear el brazo a una coordenada absoluta (X, Y, Z) conocida y establecer los *offsets* de los *encoders*. Mide 24mm y no posee desfases laterales

### 5.5.2 Grippers (v1 & v2)
Consta de un gripper pequeño con todas las piezas impresas en PLA, los ejes de rotacion son tornillos M3.
*   **v1:** Primer prototipo de agarre paralelo, impulsado por un servo y reutilizado de otro modelo de brazo robotico, mismas pinzas y mismos engranajes.
*   **v2:** Segundo prototipo cambiando el servomotor por un motor N20 conservando las mismas dimensiones del gripper. Tambien es adaptable a un motor JGA25 370.
  <img width="1600" height="1200" alt="image" src="https://github.com/user-attachments/assets/ebe1abce-4581-4abf-ae6f-87e9d08ef593" />
  <img width="595" height="393" alt="image" src="https://github.com/user-attachments/assets/cd1a9af0-6c0c-4847-99dd-bc82bd54dc5b" />
  
*   **v3:** Tercer prototipo adaptado tanto a el motor N20 y motores JGY370 de modo que el worm permite el agarre sin sostener 
  <img width="1600" height="1200" alt="image" src="https://github.com/user-attachments/assets/f8992dd4-b8a3-4be5-94a3-4d74a8c53e31" />
  

*   *(Pendiente: Renders de las garras y cinemática de los eslabones del gripper).*
*  PÖNER PLANOS DELGRIPPER 

### 5.5.3 Electromagnets (v1 & v2)
Consta de un analogo a un gripper , donde se usa como herramienta magnética para manipulación de cargas ferromagnéticas, se activa con un relé y un pin digital.
*   **v1:** Prototipo del gripper usado para agarrar objetos con la muñeca orientada a 0°
  <img width="1600" height="1200" alt="image" src="https://github.com/user-attachments/assets/e12e2941-e8a5-4aa8-a75d-7bd0570167bf" />
  <img width="1600" height="1200" alt="image" src="https://github.com/user-attachments/assets/116d2045-ccdc-49d0-b813-415d0dbd6c79" />

*   **v2:** Prototipo del gripper usado para agarrar objetos con la muñeca orientada a -90°
  <img width="1200" height="1600" alt="image" src="https://github.com/user-attachments/assets/d179bbe2-8c3c-468c-ab4c-ef708b76fa2b" />
  <img width="1200" height="1600" alt="image" src="https://github.com/user-attachments/assets/3c61cc45-fb84-40ca-af6f-1b89a8fd145e" />

## 5.6 Control Box Enclosure

Gabinete diseñado para alojar la electrónica central del brazo, separando la etapa de potencia de la lógica de control. El diseño incluye soportes para los controladores de motores (Puentes H BTS7960), conversores *Step-Down*, el arreglo de relés, una placa universal soldada con integrados, un voltimetro general y la placa principal del ESP32-S3.

<img width="1600" height="1200" alt="image" src="https://github.com/user-attachments/assets/61f34c19-4975-4715-a54f-51d810fef920" />
<img width="1600" height="1200" alt="image" src="https://github.com/user-attachments/assets/4843baa4-6d0a-42bc-9735-1c28a720f0e8" />


*   *(Pendiente: Fotos del gabinete , distribución interna de la electrónica y salidas de cables).*
* PONER PLANOS DE ESTA VAINA

* 

## 5.7 Overall Mechanical Assembly

Integración tridimensional de todos los subensambles. El modelo maestro verifica la ausencia de interferencias volumétricas (colisiones propias del CAD) a lo largo de todo el espacio de trabajo teórico del brazo robótico.
*   *(Pendiente: Render isométrico final del brazo completamente ensamblado).*

Se muestran los diversos ensambles del brazo 

Ensamble con Tool de Calibracion
<img width="439" height="367" alt="image" src="https://github.com/user-attachments/assets/318bb3e8-00fc-4f79-9edc-b9390716fea7" />

Ensamble con Tool de Gripper 
<img width="609" height="486" alt="image" src="https://github.com/user-attachments/assets/a21e4834-d202-4045-ba61-0f79d387e636" />

Ensamble con Tool de Electroiman a 0°
<img width="403" height="328" alt="image" src="https://github.com/user-attachments/assets/97747554-0758-45cf-b6a3-d1c56bb5535e" />

Ensamble con Tool de Electroiman a -90°
<img width="393" height="344" alt="image" src="https://github.com/user-attachments/assets/85d1b956-125a-4334-8c13-08ba5039d0c9" />

---

# 6. Engineering Drawings & Schematics

Esta sección contiene los planos técnicos normalizados generados a partir del modelo CAD para facilitar su fabricación, verificación y ensamblaje. Los planos incluyen tolerancias clave para agujeros pasantes y de interferencia (ajustes para insertos roscados y rodamientos).

*   *(Pendiente: Enlazar PDFs o imágenes de alta resolución de los planos 2D con vistas ortogonales, cortes y detalles de las piezas más complejas).*

---

# 7. Assembly Guide

Guía paso a paso para la construcción física del brazo robótico. Se detallan las herramientas necesarias, la tornillería empleada (M3, M4) y el orden lógico de montaje para evitar retrabajos debido a componentes inaccesibles.

## 7.1 Base Assembly
1.  *(Pendiente: Pasos de ensamble de rodamientos, base y motor J0).*
2.  *(Pendiente: Inserción de tuercas/tornillería).*

## 7.2 Shoulder Assembly
1.  *(Pendiente: Montaje del motor 5840-31ZY y brida D-Shaft).*
2.  *(Pendiente: Acople de la horquilla del hombro a la base giratoria).*

## 7.3 Elbow Joint Assembly
1.  *(Pendiente: Ensamblaje del eslabón principal, enrutamiento temprano de cables).*
2.  *(Pendiente: Fijación del motor JGY370 para el codo).*

## 7.4 Wrist Assembly
1.  *(Pendiente: Montaje del actuador final y acople de la brida universal).*

## 7.5 Tool / End-Effector Integration
1.  *(Pendiente: Procedimiento de cambio rápido de herramientas y conexión de sus respectivos cables de actuadores auxiliares).*

---

# 8. Electrical Integration & Wiring

Documentación del *hardware* electrónico, cableado de potencia y transmisión de señales lógicas. El objetivo es asegurar un enrutamiento de cables seguro que resista la fatiga mecánica del movimiento y aísle el ruido electromagnético de los motores respecto a la lógica I2C y los *encoders*.

## 8.1 Wiring & Connection Diagrams
Diagrama esquemático unifilar del sistema completo. Muestra la interconexión entre la fuente de alimentación, convertidores lógicos, placa ESP32-S3, expansor I2C (PCF8574), ADC (ADS1115), Puentes H y *encoders*.
*   *(Pendiente: Imagen del diagrama de conexiones - Fritzing, KiCad o esquemático en bloque).*

## 8.2 Soldering & Wiring
Para unir los cables de los motores se soldaron cables directamente a los de los cables propios agrupando en dos lineas distintas. Una linea con los cables del encoder trenzados y la alimentacion de estos. Otra linea para los cables de alimentacion de los motores para aislar de los encoders y evitar ruido electromagnetico generado por el cambio de sentido de corriente provocado al apagar y prender los motores por el PWM.

<img width="1200" height="1600" alt="image" src="https://github.com/user-attachments/assets/4cf4464f-2852-49ab-975f-72be2d52f2c9" />
<img width="1200" height="1600" alt="image" src="https://github.com/user-attachments/assets/01631da5-e498-4efd-93fa-f3fdfb5a97ac" />


## 8.3 Control Box Wiring
Gestión del cableado dentro del gabinete de control (*cable management*). Incluye la protección de cables con malla expansible (*cable sleeving*), terminaciones con terminales tipo férula/Dupont, y el alivio de tensión (*strain relief*) en la salida de los manguitos hacia el brazo robótico.
*   *(Pendiente: Fotos del cableado interno, diagramas de distribución de potencia desde la bornera principal).*

DESCRIBIR ESTA VAINA; ME DA MAMERA XD

# 9. Initial Testing & Validation

Una vez finalizado el ensamblaje mecánico definitivo (impresión 3D) y la integración del *hardware* electrónico, el sistema fue sometido a una batería de pruebas de validación. El propósito de esta etapa es certificar la robustez del *firmware*, evaluar el rendimiento mecánico bajo carga real y corroborar que los parámetros de operación cumplan con los límites dinámicos establecidos en la fase de diseño.

## 9.1 Unit Code Testing

Antes de ejecutar movimientos combinados que pudieran comprometer la integridad estructural, se realizaron pruebas unitarias modulares sobre las funciones críticas del *firmware* ejecutado en el ESP32-S3. Este enfoque de aislamiento permitió depurar la lógica de control sin riesgo físico.

*   **Validación de Lectura de Encoders:** Se verificó la conexion, la linealidad y la ausencia de pérdida de pasos en los *encoders* de cuadratura a máxima velocidad de PWM.
*   **Prueba de Estrés del Bus I2C:** Monitoreo continuo del ADS1115 (sensores de corriente) para descartar sobrecorrientes, cortos y malas conexiones.
*   **Auditoría de Memoria NVS:** Ciclos forzados de paradas de emergencia y cortes de energía súbitos para comprobar que los *offsets* y posiciones absolutas se recuperaban correctamente sin corromper la memoria Flash.
  
<img width="1913" height="664" alt="image" src="https://github.com/user-attachments/assets/59030da7-bc7c-47ba-91d7-207fb0eb0ee7" />


## 9.2 Physical Test Results

Las pruebas físicas se enfocaron en el comportamiento termomecánico de los actuadores y la rigidez de las piezas fabricadas en aditivo PLA. Se evaluó el desempeño del brazo bajo condiciones estacionarias y trayectorias dinámicas.

*   **Pruebas de Carga Útil (Payload):** Levantamiento sostenido de masas incrementales hasta alcanzar el límite de diseño estipulado, monitoreando la presencia de vibraciones mecánicas (cabeceo) en la articulación J0.
  
*   **Repetibilidad y Precisión (Backlash):** Medición de la desviación milimétrica del actuador final (TCP) al volver repetidamente a una coordenada base (*Home*), cuantificando la holgura en las cajas reductoras tipo Worm.
  PONER EL VIDEO QUE SE MUEVE LATERALMENTE
*   **Desempeño Térmico:** Registro de la temperatura de los motores JGY370 y 5840-31ZY tras periodos de trabajo continuo de 5 y 10 minutos.
PONER EL VIDEO QUE LE SIGUE; AHI SE VE EL BACKLASH

## 9.3 Comparison: Theoretical vs. Practical Results

# 10. Motor Control System

El esquema de control del brazo robótico se fundamenta en una arquitectura de control en cascada ejecutada asíncronamente en el microcontrolador (ESP32-S3)[cite: 2]. Esta topología se divide en un Lazo de Posición externo (que genera trayectorias espaciales) y un Lazo de Velocidad interno (que garantiza el seguimiento estricto del perfil de movimiento)[cite: 2]. Al separar ambos lazos, el sistema puede manejar las fuertes no linealidades del conjunto mecánico, compensando dinámicamente factores parasitarios como la inercia, la gravedad variable y la fricción de las cajas reductoras.

## 10.1 State Estimation & Sensor Filtering

Derivar numéricamente la posición discreta de un *encoder* de cuadratura para obtener la velocidad ($v = \frac{\Delta q}{\Delta t}$) genera un ruido de alta frecuencia inaceptable para los controladores derivativos. Para solucionar esto sin introducir el retardo de fase característico de los filtros pasa-bajos convencionales, el sistema implementa un **Filtro de Kalman Discreto (DKF)** unidimensional independiente para cada articulación[cite: 2]. 

Este estimador estocástico deduce el estado real del sistema representándolo en el vector $x = [q, \dot{q}]^T$ al minimizar iterativamente la covarianza del error de la medición.

**1. Fase de Predicción (Predict):**
El sistema proyecta el estado actual un instante de tiempo ($\Delta t$) hacia el futuro apoyándose en un modelo cinemático de velocidad constante. Se emplea la matriz de transición de estado $A$[cite: 2]:

$$A = \begin{bmatrix} 1 & dt \\ 0 & 1 \end{bmatrix} \quad \rightarrow \quad x_{\text{pred}} = A \cdot x_{\text{est}}$$

Dado que el modelo matemático no es perfecto, la incertidumbre del sistema (matriz de covarianza $P$) se expande en cada ciclo según la matriz de ruido del proceso $Q$[cite: 2]:

$$P_{\text{pred}} = A \cdot P \cdot A^T + Q$$

El *firmware* define $Q = \begin{bmatrix} 0.001 & 0 \\ 0 & 0.01 \end{bmatrix}$, asumiendo deliberadamente una mayor varianza (0.01) en la velocidad que en la posición (0.001), permitiendo que el filtro confíe más en la integral de posición geométrica que en la velocidad deducida[cite: 2].

**2. Fase de Actualización (Update):**
Al adquirir una nueva lectura angular pura del *encoder* ($z_{\text{measured}}$), el algoritmo calcula la Ganancia de Kalman ($K$)[cite: 2]. Esta ganancia pondera qué tan confiable es la predicción matemática ($P_{\text{pred}}$) frente a la medición real, la cual está afectada por el ruido de cuantización del sensor ($R = 0.01$)[cite: 2]:

$$S = H \cdot P_{\text{pred}} \cdot H^T + R$$
$$K = P_{\text{pred}} \cdot H^T \cdot S^{-1}$$

Donde $H = [1, 0]$ es la matriz de observación que extrae solo la variable medible (la posición)[cite: 2]. Finalmente, el estado estimado y su covarianza se corrigen inyectando el error residual (innovación):

$$x_{\text{est}} = x_{\text{pred}} + K \cdot (z_{\text{measured}} - H \cdot x_{\text{pred}})$$
$$P = (I - K \cdot H) \cdot P_{\text{pred}}$$

## 10.2 Firmware Control Loop Implementation

## ⚙️ Arquitectura de Control en Cascada (Firmware Control Loop)

El núcleo del movimiento del robot está gobernado por una arquitectura de **control en cascada multitasas**, orquestada por la función `CascadeControl()`[cite: 1, 2]. Este diseño garantiza el determinismo en tiempo real y una alta rigidez dinámica al separar las responsabilidades matemáticas y físicas en dos frecuencias de ejecución distintas[cite: 1, 2]:

1. **Lazos Externos (Cinemática y Trayectorias):** Se ejecutan a **100 Hz** ($dt = 10 \text{ ms}$) y calculan las velocidades objetivo de cada articulación basándose en el error de posición[cite: 1, 2].
2. **Lazo Interno (Dinámica y Esfuerzo):** Se ejecuta a **400 Hz** ($dt = 2.5 \text{ ms}$) y se encarga de que los motores alcancen físicamente las velocidades exigidas por los lazos externos[cite: 1, 2].

---

### 1. Lazo Interno: Control de Velocidad (`VelocityLoop`)

Este bucle es el responsable de interactuar directamente con el hardware[cite: 1, 2]. Convierte el error de velocidad en un ciclo de trabajo PWM ($u_{\text{PWM}}$) acotado entre 0 y 100%[cite: 1, 2]. Para lograr un movimiento fluido y preciso, el esfuerzo total no depende de un simple PI, sino de la sumatoria de cuatro modelos matemáticos[cite: 1, 2]:

$$u_{\text{PWM}} = u_{\text{ff}} + u_{\text{PI}} + u_{\text{grav}} + u_{\text{fric}}$$

** Predictor en Lazo Abierto (Feedforward):** 
Reacciona de forma instantánea a la velocidad deseada ($\dot{q}_{\text{ref}}$) sin esperar a que se acumule un error[cite: 1, 2]. Se calcula dinámicamente utilizando la constante de velocidad estimada del motor ($K_{v,\text{est}}$)[cite: 1, 2]:

$$u_{\text{ff}} = \frac{\dot{q}_{\text{ref}}}{K_{v,\text{est}}}$$

** Controlador PI ($u_{\text{PI}}$):** 
Corrige perturbaciones no modeladas[cite: 1, 2]. Para mitigar errores de discretización a altas frecuencias (400 Hz), implementa una **integración trapezoidal** e incorpora límites estrictos (*Anti-Windup*) a la integral máxima permitida[cite: 1, 2]:

$$u_{\text{PI}} = K_{p,\text{vel}} \cdot e_v + K_{i,\text{vel}} \int e_v \, dt$$

** Compensación de Gravedad ($u_{\text{grav}}$):** 
Inyecta un esfuerzo base constante para evitar que los eslabones colapsen por su propio peso[cite: 1, 2]. Es un modelo no lineal que depende de la configuración absoluta del brazo[cite: 1, 2]:

$$u_{\text{grav, J2}} = k_{\text{grav\_codo}} \cdot \cos(q_1 + q_2)$$

** Compensación de Fricción de Stribeck ($u_{\text{fric}}$):** 
Modela la asimetría de las cajas reductoras tipo *Worm*[cite: 1, 2]. Considera coeficientes independientes para movimientos a favor y en contra de la gravedad, calculando un decaimiento exponencial desde la fricción estática hacia la de Coulomb cuando el motor rompe la inercia[cite: 1, 2]:

$$F_{\text{stribeck}} = F_c + (F_s - F_c) e^{-\left(\frac{|\dot{q}_{\text{ref}}|}{\omega_s}\right)^2}$$

---

### 2. Lazos Externos: Generadores de Referencia

Los lazos externos operan a 100 Hz y su única misión es alimentar la variable $\dot{q}_{\text{ref}}$ (velocidad objetivo) del lazo interno[cite: 1, 2]. Para evitar aceleraciones mecánicas infinitas (tirones), el controlador implementa una **interpolación espaciotemporal** continua[cite: 1]. En lugar de enviar saltos bruscos de posición, el *firmware* normaliza el tiempo de la trayectoria y evalúa perfiles matemáticos de movimiento suave (como polinomios paramétricos de quinto grado o curvas senoidales) para calcular el porcentaje de avance actual y su derivada de velocidad predictiva[cite: 1, 2]. 

Dependiendo del tipo de trayectoria exigida, este avance interpolado alimenta a una de las siguientes dos estrategias matemáticas[cite: 1, 2]:

#### A. Control Cartesiano por Jacobiano Inverso (`PositionLoop`)
Se activa durante interpolaciones lineales y circulares directamente en el espacio Cartesiano de la herramienta[cite: 1, 2]. Primero, calcula una velocidad cartesiana deseada sumando el *feedforward* espacial (obtenido de la derivada de la interpolación) y un control proporcional del error XYZ[cite: 1, 2]. 

Luego, traduce esta velocidad espacial a velocidades articulares utilizando la matriz Jacobiana del manipulador[cite: 1]. Para dotar al robot de robustez y evitar solicitudes de velocidad infinitas cerca de singularidades cinemáticas, emplea la inversa de **Mínimos Cuadrados Amortiguados (Damped Least Squares - DLS)**[cite: 1]:

$$
J_{\text{DLS}} = J^T (J J^T + \lambda^2 I)^{-1}
$$

$$
\dot{q}_{\text{target}} = J_{\text{DLS}} \cdot V_{\text{cartesian}}
$$

*(Donde $\lambda^2$ es el factor de amortiguamiento configurado en el firmware)*[cite: 1].

Una característica clave de este lazo es su adaptabilidad matricial. El *firmware* gestiona la orientación de la herramienta conmutando entre tres modos de control (`control_mode`)[cite: 1]:
*   **Modo Estándar (XYZ puro):** Emplea un Jacobiano de $3 \times 4$ para controlar únicamente la posición en el espacio, dejando la orientación libre[cite: 1].
*   **Modo con Orientación (Muñeca activa):** Expande la matriz a un Jacobiano cuadrado de $4 \times 4$ añadiendo la restricción del ángulo de cabeceo ($\Phi$). Esto permite mantener la herramienta en un ángulo constante o interpolarlo suavemente respecto al suelo durante el trayecto[cite: 1].
*   **Modo de Compensación Automática:** Interpola la posición de la muñeca ($\theta_4$) de forma independiente, aplica un control proporcional sobre ella, y resta matemáticamente su efecto del vector de velocidad principal. Luego, resuelve el movimiento de los tres ejes base con un Jacobiano reducido de $3 \times 3$, asegurando que la herramienta mantenga su trayectoria sin desviarse por el giro local de la muñeca[cite: 1].

#### B. Control por Cinemática Inversa Continua (`AngleLoop`)
Se utiliza para movimientos articulares puros ('Q') o para calcular trayectorias resolviendo la Cinemática Inversa (IK) punto a punto de forma continua ('L', 'O')[cite: 1, 2]. Si durante este cálculo espacial la cinemática detecta que el punto objetivo es físicamente inalcanzable por una singularidad cruzada, el algoritmo aborta la trayectoria automáticamente por seguridad[cite: 1].

En este modo, el controlador obtiene un ángulo objetivo absoluto ($q_{\text{ref}}$) a partir de la interpolación y aplica un control proporcional sobre el error angular actual, sumado a un término de velocidad *feedforward*[cite: 1, 2]:

$$
\dot{q}_{\text{target}} = \dot{q}_{\text{ff}} + K_{p,\text{ang}} \cdot (q_{\text{ref}} - q_{\text{actual}})
$$

Para asegurar una alta precisión terminal, si el error residual es muy pequeño pero el motor no logra romper la fricción estática de la caja reductora, el lazo inyecta un pequeño pulso de velocidad direccional constante (*Joint Boost*) para forzar la entrada del eje a su zona de tolerancia estricta[cite: 1]. 

Esta velocidad resultante es acotada por los límites dinámicos del sistema y validada contra **Muros Virtuales** (que detienen el brazo si se acerca a menos de $1.5^\circ$ de sus límites mecánicos) antes de ser delegada al `VelocityLoop` subyacente para su ejecución[cite: 1].

## 10.3 Final Model Tuning & Auto-Tuning Strategy

El rendimiento dinámico óptimo de todo el esquema de control depende de la correcta parametrización de sus constantes. Debido a las tolerancias de fabricación y al desgaste mecánico, estos parámetros difieren significativamente en cada articulación [cite: 1].

Por defecto, el constructor del controlador inicializa el sistema con valores empíricos pre-calibrados [cite: 1]. Por ejemplo, los parámetros resultantes para los ejes principales son:
*   **Base (J0):** $K_p = 20.67$, $K_i = 6.17$, $F_{s,up} = 23.6\%$, $F_{s,down} = 15.8\%$ [cite: 1]
*   **Hombro (J1):** $K_p = 16.14$, $K_i = 4.82$, $F_{s,up} = 21.8\%$, $F_{s,down} = 22.4\%$ [cite: 1]

Para garantizar que el brazo mantenga este comportamiento estable tras ser desconectado, el controlador gestiona la persistencia de datos. Mediante la función `setGains()`, se actualizan los parámetros en tiempo de ejecución y se almacenan en la memoria del sistema a través de la instrucción `_sys->saveGains()` [cite: 1].

### Rutina de Auto-Sintonización (Auto-Tuning)
Para adaptar el robot al desgaste a largo plazo sin intervención humana, el firmware expone un modo de calibración automática llamado `AutoTuneLoop` [cite: 1]. Sus características operativas son:

*   **Aislamiento, Determinismo y Seguridad:** Cuando se activa la bandera de calibración (`is_tuning`), el orquestador principal detiene las operaciones normales del robot para ceder el control a esta rutina [cite: 1]. Se ejecuta a una frecuencia determinista de 200 Hz (dt = 5 ms) y monitorea constantemente las paredes virtuales (`angleLimits`) para abortar automáticamente la prueba si se alcanza un límite de seguridad [cite: 1].
*   **Mapeo de Fricción por Rampa (Zonas Muertas):** Para superar la inercia estática, el algoritmo inyecta una **rampa progresiva** de potencia PWM (a razón de 10.0 por segundo) en lugar de usar escalones [cite: 1]. Al analizar la retroalimentación de los encoders, el controlador detecta el punto exacto de ruptura cuando la velocidad supera el umbral de 0.03 rad/s, deduciendo así la fricción estática real a favor y en contra de la gravedad (`fric_up` y `fric_down`) [cite: 1].
*   **Sintonización PI (Lambda Relajado y Skogestad Modificado):** Tras hallar la fricción, el sistema inyecta un pulso escalón adicional de 20 PWM por encima de la fricción estática para evaluar la máxima velocidad en estado estacionario (`sum_amp`) [cite: 1]. Analizando esta respuesta, el sistema estima la constante de la planta ($K_v$) y aplica un método matemático relajado para mecanismos lentos (como las cajas reductoras de tornillo sin fin), calculando los parámetros óptimos mediante:
    
    $$K_p = \frac{\tau_{mec}}{K_v \cdot \tau_c}$$
    
    Para evitar oscilaciones en motores duros, se abandona la regla clásica y se extiende el tiempo de integración ($T_i$) exigiendo una estabilización suave en 0.8 segundos:
    
    $$T_i = \tau_{mec} + 4\tau_c$$
    
    Donde $K_i$ se obtiene finalmente dividiendo $K_p$ por el tiempo de integración $T_i$ [cite: 1].
*   **Aplicación en Caliente con Filtros:** Al finalizar el ciclo matemático, las ganancias se limitan dentro de rangos seguros (Kp máximo de 50.0 y Ki máximo de 80.0) [cite: 1]. Los nuevos parámetros se guardan en la m
# 11. Kinematic Modeling

# 11. Cinemática del Manipulador

Este módulo gestiona la transformación matemática entre el espacio articular (ángulos de los motores) y el espacio cartesiano (coordenadas espaciales). Todo el procesamiento matemático pesado se encapsula en la clase `Kinematic`.

## 11.1 Tabla y Modelo de Denavit-Hartenberg (DH)

Para modelar la cadena cinemática, el *firmware* utiliza la convención clásica de Denavit-Hartenberg. El sistema calcula las matrices de transformación homogénea $4\times4$ individuales para cada eslabón mediante la función `getDH(theta, alpha, d, a)`[cite: 1, 2].

La matriz genérica $A_i$ de DH implementada en el código es la siguiente[cite: 1]:

$$
A_i = 
\begin{bmatrix}
\cos(\theta) & -\cos(\alpha)\sin(\theta) & \sin(\alpha)\sin(\theta) & a\cos(\theta) \\
\sin(\theta) & \cos(\alpha)\cos(\theta) & -\sin(\alpha)\cos(\theta) & a\sin(\theta) \\
0 & \sin(\alpha) & \cos(\alpha) & d \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Según la función `angle2Pos`, los parámetros DH se aplican secuencialmente de la siguiente forma para construir los 4 eslabones[cite: 1, 2]:
1.  **Motor 0:** $\theta_1$, $\alpha = \frac{\pi}{2}$, $d = l_1$, $a = w_1$[cite: 1]. Produce la matriz `m1`[cite: 1].
2.  **Motor 1:** $\theta_2$, $\alpha = 0$, $d = w_2$, $a = l_2$[cite: 1]. Produce la matriz `m2`[cite: 1].
3.  **Motor 2:** $\theta_3$, $\alpha = 0$, $d = w_3$, $a = l_3$[cite: 1]. Produce la matriz `m3`[cite: 1].
4.  **Motor 3:** $\theta_4$, $\alpha = 0$, $d = w_4$, $a = l_4$[cite: 1]. Produce la matriz `m4`[cite: 1].

## 11.2 Cinemática Directa (Forward Kinematics - FK)

La Cinemática Directa (FK) halla la posición final ($X, Y, Z$) del efector conociendo los ángulos de los 4 motores ($\theta_1, \theta_2, \theta_3, \theta_4$)[cite: 1]. 

El sistema obtiene la pose multiplicando secuencialmente las matrices DH calculadas previamente[cite: 1]:

$$T_{TCP}^{Base} = m_1 \cdot m_2 \cdot m_3 \cdot m_4$$

Esta operación la realiza la función `angle2Pos`, retornando la matriz resultante en `resMatrix`[cite: 1]. De la última columna de esta matriz, el sistema maestro extraerá posteriormente las coordenadas $X, Y, Z$.

## 11.3 Cinemática Inversa (Inverse Kinematics - IK)

La Cinemática Inversa calcula los ángulos articulares requeridos ($\theta_1, \theta_2, \theta_3, \theta_4$) dadas las coordenadas espaciales ($X, Y, Z$)[cite: 1, 2]. El código provee dos implementaciones geométricas analíticas mediante la sobrecarga de la función `pos2Angle()`. Ambas asumen una compensación base en el plano horizontal (offset lateral $W = w_2 + w_3 + w_4$)[cite: 1, 2].

### Cálculo de la Base (Eje 1) para ambos modos
Independiente del modo, el ángulo de la base ($t_1$) se halla proyectando las coordenadas al plano $XY$[cite: 1]:

1.  Se verifica que el punto no esté dentro de la zona muerta lateral del offset: $r_{xy}^2 = X^2 + Y^2 \geq W^2$[cite: 1].
2.  El ángulo de la base se calcula considerando la tangente de la posición y el offset lateral:
    $$t_1 = \arctan\left(\frac{Y}{X}\right) + \arctan\left(\frac{W}{\sqrt{r_{xy}^2 - W^2}}\right)$$[cite: 1]

Tras esto, las coordenadas se proyectan a un marco de referencia cilíndrico relativo, definido por la altura desde la base $l_1$ y el radio planar efectivo $R$[cite: 1]:
*   $z_{Rel} = Z - l_1$[cite: 1]
*   $R = X\cos(t_1) + Y\sin(t_1) - w_1$[cite: 1]

### Modo A: Muñeca Libre (Orientación no controlada / 3 DOF)
En este régimen, se reciben las coordenadas deseadas y un ángulo estático opcional `theta4_in` (normalmente 0)[cite: 1, 2]. Se utiliza cuando la inclinación global de la herramienta no importa.

El algoritmo agrupa matemáticamente el eslabón 3 ($l_3$) y el eslabón 4 ($l_4$) en un único eslabón virtual $L_{virt}$[cite: 1]:
$$L_{virt} = \sqrt{l_3^2 + l_4^2 + 2l_3 l_4 \cos(\theta_4)}$$[cite: 1]

Con $L_{virt}$ y $l_2$, el sistema aplica la Ley de los Cosenos sobre el triángulo formado para hallar la distancia normalizada $D$[cite: 1]:
$$D = \frac{R^2 + z_{Rel}^2 - l_2^2 - L_{virt}^2}{2 l_2 L_{virt}}$$[cite: 1]

Finalmente, resolviendo la geometría (donde el signo depende del booleano `elbowUp`)[cite: 1]:
*   $\gamma = \arctan2(\pm \sqrt{1 - D^2}, D)$[cite: 1]
*   $\beta = \arctan2(l_4 \sin(\theta_4), l_3 + l_4 \cos(\theta_4))$[cite: 1]
*   **Articulación 3:** $t_3 = \gamma - \beta$[cite: 1]
*   **Articulación 2:** $t_2 = \arctan2(z_{Rel}, R) - \arctan2(L_{virt} \sin(\gamma), l_2 + L_{virt} \cos(\gamma))$[cite: 1]

### Modo B: Muñeca Automática (Orientación controlada / 4 DOF)
En este modo, se recibe un parámetro de inclinación de la herramienta ($\Phi$ o `phi_val`), el cual define el ángulo del efector final relativo al suelo ($t_2 + t_3 + t_4 = \Phi$)[cite: 1].

Aquí, el algoritmo "retrocede" desde el efector final usando $\Phi$ para encontrar la posición de la articulación de la muñeca ($R_j, z_j$)[cite: 1]:
*   $R_j = R - l_4\cos(\Phi)$[cite: 1]
*   $z_j = z_{Rel} - l_4\sin(\Phi)$[cite: 1]

Luego, aplica la Ley de los Cosenos solo con $l_2$ y $l_3$ para alcanzar la muñeca[cite: 1]:
$$D = \frac{R_j^2 + z_j^2 - l_2^2 - l_3^2}{2 l_2 l_3}$$[cite: 1]

Resolviendo los ángulos finales (con el signo dado por `elbowUp`)[cite: 1]:
*   **Articulación 3:** $t_3 = \arctan2(\pm \sqrt{1 - D^2}, D)$[cite: 1]
*   **Articulación 2:** $t_2 = \arctan2(z_j, R_j) - \arctan2(l_3 \sin(t_3), l_2 + l_3 \cos(t_3))$[cite: 1]
*   **Articulación 4:** Despejada de la restricción de orientación: $t_4 = \Phi - (t_2 + t_3)$[cite: 1]
---

# 12. Motion Control & Command Reference

El control de trayectoria del brazo permite definir cómo se mueve de un punto $A$ a un punto $B$. El sistema utiliza perfiles de velocidad suavizados (Quínticos/Jerk-Free por defecto `traj_profile = 'i'`) para no estresar mecánicamente las transmisiones[cite: 2, 3].

La parametrización de interpolación temporal $s(\tau)$ donde $\tau = \frac{t_{\text{actual}}}{t_{\text{total}}}$ se modela así[cite: 2]:

$$s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5 \quad \rightarrow \quad \frac{ds}{d\tau} = \frac{30\tau^2(1 - 2\tau + \tau^2)}{t_{\text{total}}}$$

## 12.1 Joint-Space Configuration (`MovA`)

El comando `movA` genera una trayectoria puramente articular. El brazo viaja desde $[q_{\text{start}}]$ hasta $[q_{\text{final}}]$ interpolando directamente los grados de los motores[cite: 2]. 

$$q_{\text{ref}}(t) = q_{\text{start}} + (q_{\text{final}} - q_{\text{start}}) \cdot s(\tau)$$

El TCP trazará una curva impredecible en el espacio cartesiano, pero garantiza el movimiento más suave y rápido posible para los motores, usado generalmente para ir a posturas de descanso[cite: 2].

## 12.2 Unconstrained Point-to-Point Motion (`MovJ`)

El comando `movJ` recibe coordenadas cartesianas ($X, Y, Z$), utiliza la Cinemática Inversa (`pos2Angle()`) para evaluar los ángulos destino *una sola vez*, y luego ejecuta una interpolación articular idéntica a `MovA`[cite: 2]. Al igual que `MovA`, no garantiza una línea recta en 3D, pero evita las altas exigencias computacionales en tiempo real[cite: 2].

## 12.3 Linear Interpolated Motion (`MovL` & `MovLJc`)

Ambos comandos fuerzan a que la herramienta trace una **línea recta estricta** en el espacio cartesiano 3D, pero utilizan estrategias matemáticas totalmente distintas que condicionan su rendimiento en tiempo real.

**1. `MovL` (Línea por IK Continua):**[cite: 2, 3]
Interpola las coordenadas $[X, Y, Z]$ paso a paso. En cada interrupción del lazo cartesiano ($dt = 10\text{ ms}$), invoca a la cinemática inversa algebraicamente para hallar el $q$ correspondiente[cite: 2].
*   *Ventaja:* Posicionamiento final absoluto y exacto[cite: 2].
*   *Desventaja:* Si la línea atraviesa una singularidad, las derivadas articulares tienden al infinito y los motores colapsan algebraicamente (el código lo previene deteniendo el brazo)[cite: 2].

**2. `MovLJc` (Línea por Jacobiano Diferencial DLS):**[cite: 2, 3]
En lugar de buscar ángulos absolutos, genera velocidades cartesianas ($\dot{X}, \dot{Y}, \dot{Z}$) y las traduce a velocidades articulares ($\dot{q}$) utilizando la matriz Jacobiana $J_{3\times4}$[cite: 2]. Para evitar la división por cero en singularidades, no utiliza una matriz pseudoinversa estándar, sino el algoritmo **DLS (Damped Least Squares)**[cite: 2]:

$$V_{\text{cartesian}} = \left( \Delta X \cdot \frac{ds}{dt} \right) + K_{p,\text{pos}} (X_{\text{target}} - X_{\text{current}})$$
$$\dot{q}_{\text{target}} = J^T \cdot (J \cdot J^T + \lambda^2 I)^{-1} \cdot V_{\text{cartesian}}$$

*   *Ventaja:* El término de amortiguamiento numérico ($\lambda^2 = 0.01$) hace que el sistema sea extremadamente robusto; la matriz nunca se vuelve singular[cite: 2]. El brazo pasa cerca de la singularidad disminuyendo su velocidad sin crashear.
*   *Desventaja:* El integrador numérico genera un leve desvío (error de arrastre) que debe ser absorbido por la ganancia Proporcional Cartesiana ($K_{p,\text{pos}}$)[cite: 2, 3].

## 12.4 Circular Interpolated Motion (`MovC` & `MovCJc`)

Para los arcos y círculos, se requieren 3 puntos en el espacio ($P_1$: Inicio, $P_2$: Vía, $P_3$: Fin)[cite: 2, 3]. El sistema define un plano espacial utilizando producto cruz[cite: 2]:

$$u = P_2 - P_1 \quad ; \quad w = P_3 - P_1 \quad \rightarrow \quad N = u \times w$$

Esto permite hallar el centro geométrico ($C$), el radio ($R$) y los vectores ortonormales bidimensionales del plano de giro ($\vec{v}_x, \vec{v}_y$)[cite: 2]. La trayectoria interpolada se calcula barriendo el ángulo total $\theta$:

$$X_{\text{ref}}(\tau) = C + R \cdot \cos(\theta \cdot s(\tau)) \cdot \vec{v}_x + R \cdot \sin(\theta \cdot s(\tau)) \cdot \vec{v}_y$$

Al igual que en los movimientos lineales, `MovC` resuelve esta ecuación en cada instante utilizando la Cinemática Inversa (IK), mientras que `MovCJc` deriva esta ecuación para inyectar vectores tangenciales de velocidad directamente al controlador Jacobiano DLS[cite: 2, 3].

# 13. Caracterización del Sistema (System Characterization)

Esta sección consolida las especificaciones físicas, eléctricas y operativas del brazo robótico, así como las directrices necesarias para su correcta manipulación y puesta en marcha por parte del usuario final.

## 13.1 Ficha Técnica (Technical Datasheet)

El manipulador robótico ha sido diseñado con las siguientes especificaciones técnicas principales. 

<!-- TABLA PRINCIPAL DEL SISTEMA -->
<table>
  <thead>
    <tr>
      <th colspan="4" style="background-color: #FF8C00; color: white; text-align: left; font-size: 16px;">ABSOLUTE MAXIMUM RATINGS / SYSTEM DATASHEET</th>
    </tr>
    <tr>
      <th style="background-color: #f2f2f2; text-align: left; width: 40%;">PARÁMETRO</th>
      <th style="background-color: #f2f2f2; text-align: center; width: 25%;">ESPECIFICACIÓN</th>
      <th style="background-color: #f2f2f2; text-align: center; width: 15%;">UNIDAD</th>
      <th style="background-color: #f2f2f2; text-align: center; width: 20%;">RESOLUCIÓN</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><strong>Grados de Libertad (DOF)</strong></td>
      <td align="center">4</td>
      <td align="center">-</td>
      <td align="center">-</td>
    </tr>
    <tr>
      <td><strong>Carga Útil Máxima (Payload)</strong></td>
      <td align="center">150</td>
      <td align="center">g</td>
      <td align="center">-</td>
    </tr>
    <tr>
      <td><strong>Alcance Máximo Radial (Reach)</strong></td>
      <td align="center">630.03</td>
      <td align="center">mm</td>
      <td align="center">-</td>
    </tr>
    <tr>
      <td><strong>Alimentación Eléctrica Lógica</strong></td>
      <td align="center">3.3</td>
      <td align="center">V DC</td>
      <td align="center">-</td>
    </tr>
    <tr>
      <td><strong>Motor JGY 370</strong></td>
      <td align="center">12</td>
      <td align="center">V DC</td>
      <td align="center">12.5 A Máx.</td>
    </tr>
    <tr>
      <td><strong>Motor 5840 31ZY</strong></td>
      <td align="center">12</td>
      <td align="center">V DC</td>
      <td align="center">12.5 A Máx.</td>
    </tr>
    <tr>
      <td><strong>Encoder Cuadratura Hall JGY</strong></td>
      <td align="center">16</td>
      <td align="center">CPR</td>
      <td align="center"></td>
    </tr>
     <td><strong>Encoder Cuadratura Hall 31ZY</strong></td>
      <td align="center">5</td>
      <td align="center">CPR</td>
      <td align="center"></td>
    </tr>
    <tr>
      <td><strong>Controlador Principal</strong></td>
      <td align="center">ESP32</td>
      <td align="center">S3</td>
      <td align="center">N16R(</td>
    </tr>
    <tr>
      <td><strong>Frecuencia de Control (Lazo PID)</strong></td>
      <td align="center">100 / 400</td>
      <td align="center">Hz</td>
      <td align="center">$\Delta t$: 10ms / 2.5ms</td>
    </tr>
    <tr>
      <td><strong>Peso Total del Sistema</strong></td>
      <td align="center">1.4</td>
      <td align="center">kg</td>
      <td align="center"></td>
    </tr>
  </tbody>
</table>

<br>

<!-- TABLA: ELECTROIMÁN 1 (HORIZONTAL) -->
<table>
  <thead>
    <tr>
      <th colspan="4" style="background-color: #FF8C00; color: white; text-align: left; font-size: 16px;">END-EFFECTOR DATASHEET: ELECTROIMÁN HORIZONTAL</th>
    </tr>
    <tr>
      <th style="background-color: #f2f2f2; text-align: left; width: 40%;">PARÁMETRO</th>
      <th style="background-color: #f2f2f2; text-align: center; width: 25%;">ESPECIFICACIÓN</th>
      <th style="background-color: #f2f2f2; text-align: center; width: 15%;">UNIDAD</th>
      <th style="background-color: #f2f2f2; text-align: center; width: 20%;">RESOLUCIÓN</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><strong>Voltaje de Operación Nominal</strong></td>
      <td align="center">12</td>
      <td align="center">V DC</td>
      <td align="center">-</td>
    </tr>
    <tr>
      <td><strong>Consumo de Corriente Máximo</strong></td>
      <td align="center">300</td>
      <td align="center">mA</td>
      <td align="center">-</td>
    </tr>
    <tr>
      <td><strong>Fuerza de Sujeción</strong></td>
      <td align="center">4.905</td>
      <td align="center">N</td>
      <td align="center">~</td>
    </tr>
    <tr>
      <td><strong>Peso de la Herramienta</strong></td>
      <td align="center">60</td>
      <td align="center">g</td>
      <td align="center"></td>
    </tr>
         <tr>
      <td><strong>Longitud Total)</strong></td>
      <td align="center">60 tcp/80</td>
      <td align="center">mm</td>
      <td align="center"></td>
    </tr>
  </tbody>
</table>

<br>

<!-- TABLA: ELECTROIMÁN 2 (VERTICAL) -->
<table>
  <thead>
    <tr>
      <th colspan="4" style="background-color: #FF8C00; color: white; text-align: left; font-size: 16px;">END-EFFECTOR DATASHEET: ELECTROIMÁN VERTICAL</th>
    </tr>
    <tr>
      <th style="background-color: #f2f2f2; text-align: left; width: 40%;">PARÁMETRO</th>
      <th style="background-color: #f2f2f2; text-align: center; width: 25%;">ESPECIFICACIÓN</th>
      <th style="background-color: #f2f2f2; text-align: center; width: 15%;">UNIDAD</th>
      <th style="background-color: #f2f2f2; text-align: center; width: 20%;">RESOLUCIÓN</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><strong>Voltaje de Operación Nominal</strong></td>
      <td align="center">[Valor]</td>
      <td align="center">V DC</td>
      <td align="center">-</td>
    </tr>
    <tr>
      <td><strong>Consumo de Corriente Máximo</strong></td>
      <td align="center">300</td>
      <td align="center">mA</td>
      <td align="center">-</td>
    </tr>
    <tr>
      <td><strong>Fuerza de Sujeción</strong></td>
      <td align="center">4.905</td>
      <td align="center">N</td>
      <td align="center">-</td>
    </tr>
    <tr>
      <td><strong>Peso de la Herramienta</strong></td>
      <td align="center">55</td>
      <td align="center">g</td>
      <td align="center"></td>
    </tr>
    <tr>
      <td><strong>Longitud Total)</strong></td>
      <td align="center">60</td>
      <td align="center">mm</td>
      <td align="center"></td>
    </tr>
  </tbody>
</table>

<br>

<!-- TABLA: GRIPPER -->
<table>
  <thead>
    <tr>
      <th colspan="4" style="background-color: #FF8C00; color: white; text-align: left; font-size: 16px;">END-EFFECTOR DATASHEET: GRIPPER (PINZA)</th>
    </tr>
    <tr>
      <th style="background-color: #f2f2f2; text-align: left; width: 40%;">PARÁMETRO</th>
      <th style="background-color: #f2f2f2; text-align: center; width: 25%;">ESPECIFICACIÓN</th>
      <th style="background-color: #f2f2f2; text-align: center; width: 15%;">UNIDAD</th>
      <th style="background-color: #f2f2f2; text-align: center; width: 20%;">RESOLUCIÓN</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><strong>Voltaje de Operación</strong></td>
      <td align="center">12</td>
      <td align="center">V DC</td>
      <td align="center">-</td>
    </tr>
    <tr>
      <td><strong>Apertura Máxima de Mandíbulas</strong></td>
      <td align="center">70</td>
      <td align="center">mm</td>
      <td align="center">-</td>
    </tr>
    <tr>
      <td><strong>Fuerza de Agarre (Aprox)</strong></td>
      <td align="center">3</td>
      <td align="center">N</td>
      <td align="center">-</td>
    </tr>
    <tr>
      <td><strong>Peso de la Herramienta</strong></td>
      <td align="center">220</td>
      <td align="center">g</td>
      <td align="center">Con motor incluido</td>
    </tr>
    <tr>
      <td><strong>Longitud Total)</strong></td>
      <td align="center">105 / 130</td>
      <td align="center">mm</td>
      <td align="center"></td>
    </tr>
  </tbody>
</table>

<br>

<!-- TABLA: PUNTA DE CALIBRACIÓN -->
<table>
  <thead>
    <tr>
      <th colspan="4" style="background-color: #FF8C00; color: white; text-align: left; font-size: 16px;">END-EFFECTOR DATASHEET: PUNTA DE CALIBRACIÓN</th>
    </tr>
    <tr>
      <th style="background-color: #f2f2f2; text-align: left; width: 40%;">PARÁMETRO</th>
      <th style="background-color: #f2f2f2; text-align: center; width: 25%;">ESPECIFICACIÓN</th>
      <th style="background-color: #f2f2f2; text-align: center; width: 15%;">UNIDAD</th>
      <th style="background-color: #f2f2f2; text-align: center; width: 20%;">RESOLUCIÓN</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><strong>Longitud Total)</strong></td>
      <td align="center">24</td>
      <td align="center">mm</td>
      <td align="center">Medido en Eje Z TCP</td>
    </tr>
    <tr>
      <td><strong>Peso de la Herramienta</strong></td>
      <td align="center">5</td>
      <td align="center">g</td>
      <td align="center">-</td>
    </tr>
  </tbody>
</table>

## 13.2 Instruction Manual
# Manual de Referencia Técnica - Instrucciones, funciones y tipos de datos

---
**RobotWare Base / ControlManager**
**Revisión: 1.0**

---

# 1 Instrucciones de Movimiento (mov)

Este documento detalla las instrucciones de movimiento implementadas en el sistema `ControlManager` para el control de trayectorias articulares y cartesianas del robot [cite: 1, 2].

---

## 1.1 `movA` - Mueve el robot en el espacio articular
*ControlManager Base*

---
### Utilización
`movA` se utiliza para trasladar los ejes del robot de forma síncrona hacia una posición articular de destino especificada en radianes [cite: 1]. Durante el movimiento, no se garantiza una trayectoria en línea recta del punto central de la herramienta (TCP). La duración de la trayectoria se calcula automáticamente basándose en la diferencia de posición máxima y la velocidad angular configurada [cite: 1].

### Ejemplos básicos
**Ejemplo 1**
```cpp
robot.movA(0.0, 1.57, -1.57, 0.0, 20.0);
```
El TCP de la herramienta, se mueve hacia la posición articular de destino con una velocidad máxima de `20.0` grados/segundo.

---
### Argumentos
`movA(q1, q2, q3, q4, speed_deg_s)`

<dl>
  <dt><code>q1, q2, q3, q4</code></dt>
  <dd>
    <em>Tipo de dato:</em> <code>float</code><br>
    Posición de destino absoluta para cada una de las 4 articulaciones (en radianes). El sistema verifica automáticamente que estos valores se encuentren dentro de los límites físicos configurados (<code>_min_angles</code> y <code>_max_angles</code>) [cite: 1, 2].
  </dd>

  <dt><code>speed_deg_s</code></dt>
  <dd>
    <em>Tipo de dato:</em> <code>float</code><br>
    Velocidad máxima angular solicitada en grados por segundo [cite: 1]. Se limita internamente entre <code>0.05 rad/s</code> y <code>_max_vel_rad</code> (30°/s) por seguridad [cite: 1].
  </dd>
</dl>

> ℹ️ **Nota de Seguridad**
> Si alguna de las coordenadas articulares excede las paredes virtuales, la trayectoria no se iniciará o será abortada automáticamente [cite: 1].

---
<br>

## 1.2 `movJ` - Mueve el robot a una posición cartesiana (Articular Pura)
*ControlManager Base*

---
### Utilización
`movJ` traslada el TCP a una coordenada cartesiana de destino (X, Y, Z). Calcula los ángulos articulares requeridos mediante Cinemática Inversa (IK) solo al inicio y luego interpola de forma articular (no lineal en el espacio cartesiano) [cite: 1]. Es ideal para movimientos rápidos donde la trayectoria exacta del TCP no es crítica.

---
### Argumentos
`movJ(x, y, z, speed_mm_s, [phi_val], [elbowUp])`

<dl>
  <dt><code>x, y, z</code></dt>
  <dd>
    <em>Tipo de dato:</em> <code>float</code><br>
    Coordenadas de destino del TCP en milímetros [cite: 1].
  </dd>

  <dt><code>speed_mm_s</code></dt>
  <dd>
    <em>Tipo de dato:</em> <code>float</code><br>
    Velocidad lineal deseada del TCP en milímetros por segundo [cite: 1].
  </dd>

  <dt><code>[phi_val]</code> (Opcional)</dt>
  <dd>
    <em>Tipo de dato:</em> <code>float</code><br>
    Ángulo de orientación final deseado del actuador en radianes [cite: 1, 2].
  </dd>

  <dt><code>[elbowUp]</code> (Opcional, por defecto <code>true</code>)</dt>
  <dd>
    <em>Tipo de dato:</em> <code>bool</code><br>
    Selecciona la configuración cinemática (codo arriba / codo abajo) para resolver las múltiples soluciones de la cinemática inversa [cite: 1, 2].
  </dd>
</dl>

---
<br>

## 1.3 `movL` / `movLJc` - Mueve el robot linealmente
*Trayectorias Cartesianas*

---
### Utilización
Estas instrucciones trasladan el TCP en línea recta desde la posición actual hacia el destino.
*   **`movL`**: Utiliza Cinemática Inversa (IK) Continua durante toda la trayectoria interpolada. Si el robot cruza una singularidad matemática, el movimiento se aborta [cite: 1].
*   **`movLJc`**: Utiliza control Jacobiano DLS (Damped Least Squares) para calcular las velocidades articulares [cite: 1]. Proporciona una mejor estabilidad cerca de las singularidades, introduciendo un factor de amortiguamiento `lambda_sq = 0.01f` [cite: 1].

### Ejemplos básicos
**Ejemplo 1**
```cpp
// Movimiento lineal estricto IK
robot.movL(150.0, 50.0, 100.0, 25.0); 

// Movimiento lineal por Jacobiano DLS con orientación específica
robot.movLJc(150.0, 50.0, 100.0, 25.0, 1.57);
```

---
### Argumentos
`movL(x, y, z, speed_mm_s, [phi_rad], [elbowUp])`
`movLJc(x, y, z, speed_mm_s, [phi_rad])`

<dl>
  <dt><code>speed_mm_s</code></dt>
  <dd>
    Velocidad lineal de traslación del TCP. Se asume como un parámetro de velocidad constante.
  </dd>

  <dt><code>phi_rad</code></dt>
  <dd>
    Especifica la orientación final. <code>movLJc</code> calculará una matriz Jacobiana de 4x4 o 3x4 dependiendo de si se requiere controlar el ángulo <code>phi</code> (<code>control_mode = 1</code>) o solo la posición [cite: 1, 2].
  </dd>
</dl>

---
<br>

## 1.4 `movC` / `movCJc` - Mueve el robot en círculo
*Trayectorias Cartesianas*

---
### Utilización
Se utilizan para trasladar el TCP a través de un recorrido circular. El círculo se define a partir de la posición inicial actual, un punto de paso o de vía (`via`) y el punto de destino (`end`) [cite: 1, 2].
Al igual que en movimientos lineales, `movC` emplea IK continua [cite: 1], mientras que `movCJc` emplea el integrador Jacobiano DLS [cite: 1].

### Ejecución de programas
El radio, el centroide, y los vectores directores (`circ_vx`, `circ_vy`) se calculan resolviendo la intersección normalizada de los vectores generados por los tres puntos. Si los puntos son colineales (`normN2 < 1e-6f`), la instrucción retornará `false` evitando un error matemático [cite: 1]. 
La rotación total (`circ_total_angle`) puede alcanzar hasta `2.0 * M_PI` radianes [cite: 1].

### Ejemplos básicos
**Ejemplo 1**
```cpp
// Se desplaza en círculo pasando por (100, 50, 100) hasta (50, 0, 100)
robot.movC(100.0, 50.0, 100.0, 50.0, 0.0, 100.0, 30.0);
```

---
### Argumentos
`movC(x_via, y_via, z_via, x_end, y_end, z_end, speed_mm_s, [phi_rad], [elbowUp])`

<dl>
  <dt><code>x_via, y_via, z_via</code></dt>
  <dd>
    <em>Tipo de dato:</em> <code>float</code><br>
    El punto de paso (vía) en milímetros que define la curvatura del círculo [cite: 1, 2].
  </dd>

  <dt><code>x_end, y_end, z_end</code></dt>
  <dd>
    <em>Tipo de dato:</em> <code>float</code><br>
    El punto de destino final del arco o círculo [cite: 1, 2].
  </dd>
</dl>

> ℹ️ **Nota de Interpolación**
> La interpolación por defecto durante la ejecución se rige mediante polinomios. El perfil activo se define internamente por la variable `traj_profile` [cite: 2], que puede ser 's' (senoidal), 'c' (cúbico), o 'i' (quintico) [cite: 1].

---
<br>

## 1.5 `movZ` / `movZJc` - Movimiento Vertical Exclusivo
*Trayectorias Especiales*

---
### Utilización
Ejecuta un movimiento rectilíneo modificando únicamente la coordenada Z, manteniendo constantes las posiciones X e Y actuales extraídas mediante Cinemática Directa de los sensores [cite: 1].

---
### Argumentos
`movZ(z, speed_mm_s, phi_rad)`

<dl>
  <dt><code>z</code></dt>
  <dd>
    Nueva cota Z en milímetros [cite: 2].
  </dd>
</dl>
Las posiciones X e Y se heredan de `_kinematic->angle2Pos(sensor_q)` [cite: 1].
---

# 14. Evaluación de Rendimiento y Pruebas Experimentales (Performance Evaluation & Experimental Tests)

Para validar el desempeño del diseño de control y las formulaciones cinemáticas implementadas, se llevaron a cabo diversas pruebas experimentales. Se analizó la precisión de posicionamiento y el comportamiento dinámico del robot bajo distintas estrategias de control y perfiles de trayectoria.

## 14.1 Rendimiento de Posicionamiento Angular (Angular Positioning Performance)

Se evaluó la capacidad del controlador PID en cascada para alcanzar y mantener referencias articulares estáticas. Se enviaron comandos de paso (*step*) a cada articulación ($q_0, q_1, q_2, q_3$) y se registró la respuesta transitoria y el error en estado estable. 

* *(Insertar aquí descripción de la respuesta: tiempo de asentamiento, sobreimpulso, y el error estático residual medio en grados).*
* *(Añadir referencia a una gráfica de respuesta al escalón articular).*

Al llegar a una posicion, se evidencia numericamente como llega a un valor casi exacto, apenas limitado por la tolerancia permitida de las cajas de 0.0081° de los JGY y de 0.0152 del motor 31ZX.
<img width="408" height="210" alt="image" src="https://github.com/user-attachments/assets/124c4c81-5937-430d-89cb-3be643ef05ca" />


## 14.2 Trayectoria de Movimiento en el Espacio Libre (Free-Space Motion Trajectory)

Esta prueba evaluó los movimientos de tipo Point-to-Point (PTP) utilizando interpolación articular pura con el comando MovJ. Se verificó que el sistema generara perfiles de velocidad suaves interpoladas sin saturar los actuadores, garantizando un desplazamiento fluido entre posiciones distantes del espacio de trabajo sin seguir una ruta cartesiana estricta como se muestra en el siguiente video.

VIDEO MJ 

TABLA

[Imagen](https://github.com/labsir-un/Robotica-2026-I-Equipo-3E-Diaz-Pulido/tree/main/Proyecto%20Final%20Robotica)
Cada cuadro tiene lados de 100mm 

Se evidencia un desfase significativo en el eje X base de 15mm que aumenta progresivamente con la lejania del origen, el desfase en Y es menor aunque tiene un comportamiento similar. El resultado consta de poligonos que aumentan el angulo respecto a cuanto mas se alejan, lo que se explica por el error de Backlash de los motores usados que ronda entre 2 a 5° dependiendo del ajuste del prisionero. Sin embargo si se conservan las distancias entre puntos si se elimina ese desfase. Otro error importante es el de calibracion del cero inicial dado a que este añade desfase a los angulos y añade mayor deformacion a las trayectorias. 

## 14.3 Precisión en el Seguimiento de Línea (Line-Following Accuracy)

Se programó el efector final para trazar una trayectoria rectilínea entre el punto (300 0 5) y el punto (300 -300 5) en el espacio tridimensional. El objetivo es evaluar la desviación cartesiana del efector respecto a la línea ideal. Se trazo una linea respecto a los dos puntos y se compararon los dos controladores. 

### 14.3.1 Control en el Espacio Articular (Joint-Space Control)
La línea fue discretizada en múltiples puntos (*via-points*) calculados con cinemática inversa estática. Entre cada punto, las articulaciones se movieron mediante interpolación articular como se muestra en el siguiente video.

VIDEO ML

Se evidencia que si bien intenta seguir la linea recta marcada por la regla y el error no es tan significativo , el backlash de los motores es importante y afecta la medicion. 

### 14.3.2 Control Diferencial Jacobiano (Jacobian Differential Control)
Se ejecutó la misma recta implementando el método de la matriz Jacobiana. El vector de velocidad cartesiana se tradujo continuamente en velocidades articulares como se muestra en el video.

VIDEO MLJ

Se evidencia que ambas lineas son similares, aunque el seguimiento del jacobiano MovLJc es mucho mas inestable que el MovL anterior. Tambien en pruebas fuera de camara sucede que el jacobiano se demora en encontrar la velocidad ideal, llega a una posicion lejana y se acerca posteriormente al punto. 

## 14.4 Precisión en el Seguimiento de Círculos (Circle-Following Accuracy)

El manipulador fue comandado para dibujar un círculo en el plano tridimensional, definido por un punto inicial, un punto de paso y un punto final, evaluando el error de contorno. 
El circulo a evaluar es: 
Punto de inicio (0°): X=350, Y=-200
Tramo 1 (0° a 120°): Pasa por 60° (Vía) y termina en 120° (Fin)
Tramo 2 (120° a 240°): Pasa por 180° (Vía) y termina en 240° (Fin)
Tramo 3 (240° a 360°): Pasa por 300° (Vía) y termina en 360° / 0° (Fin)

### 14.4.1 Control en el Espacio Articular (Joint-Space Control)
La trayectoria circular fue subdividida algorítmicamente en pequeños segmentos lineales/angulares resolviendo la cinemática inversa analítica en cada instante de muestreo. Se muestra el siguiente video de una trayectoria circular.  
Los comandos usados configurados en el terminal que llaman a MovC son: 
ML 350.0 -200.0 5.0 30.0
MC 325.0 -156.7 5.0 275.0 -156.7 5.0 30.0
MC 250.0 -200.0 5.0 275.0 -243.3 5.0 30.0
MC 325.0 -243.3 5.0 350.0 -200.0 5.0 30.0

VIDEO C

### 14.4.2 Control Diferencial Jacobiano (Jacobian Differential Control)
Las velocidades cartesianas tangenciales y normales del círculo fueron proyectadas directamente a los motores utilizando la matriz Jacobiana del manipulador. Se muestra el siguiente video de una trayectoria circular. 
Los comandos usados configurados en el terminal que llaman a MovCJc son: 
MJ 350.0 -200.0 5.0 30.0
MCJ 325.0 -156.7 5.0 275.0 -156.7 5.0 30.0
MCJ 250.0 -200.0 5.0 275.0 -243.3 5.0 30.0
MCJ 325.0 -243.3 5.0 350.0 -200.0 5.0 30.0

VIDEO CJ

## 14.5 Validación de Pick-and-Place (Pick-and-Place Validation)

Para probar la aplicabilidad del robot en tareas industriales estándar, se diseñó una rutina continua de manipulación de objetos utilizando un actuador electromagnético, midiendo la repetibilidad y el control de orientación de la herramienta.

### 14.5.1 Ejecución con Electroimán Vertical (Vertical Electromagnet Execution)
Se requirió que el robot tomara un motor en una posicion dada, gira la muñeca, mueve a un punto elegido y la deposita en otra posicion . Para ello, se activó el modo "Muñeca Forzada" (3 DOF), forzando una restricción de cabeceo (*Pitch* constante, $\Phi$). Ademas de otra prueba donde transporte un motor sosteniendo el angulo de cabeceo constante en toda la trayectoria como se muestran en los siguientes videos. 

Video 1

Video 2

La prueba demostró que el controlador cinemático logró mantener la orientación del efector estricta durante toda la traslación cartesiana, evitando la caída de la pieza magnética y asegurando una aproximación precisa a la superficie objetivo.
