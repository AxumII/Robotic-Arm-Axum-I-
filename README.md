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

> *(Insertar aquí gráficas arrojadas por los scripts evaluar_barrido_multi_material y evaluar_deformacion_seccion)*

## 3.5 Bill of Materials (BOM) & Component Selection

Con base en los resultados del remanente de torque y la viabilidad del mercado, se establece la siguiente lista de componentes principales requeridos para satisfacer la dinámica del sistema:

*   **Actuadores:** Motorreductores DC tipo Worm/Planetario equipados con encoders de cuadratura para el cierre del lazo de control de posición.
*   **Microcontrolador (Hardware-in-the-Loop):** ESP32-S3 (N16R8) para gestionar las interrupciones de los encoders, la matemática de coma flotante de las matrices cinemáticas y la generación de señales de control PWM.
*   **Interfaz de Usuario (HMI):** Pantalla ST7789 vía SPI integrada con joysticks analógicos y un teclado matricial para el modo de operación manual y rutinas de calibración.
*   **Mecánica y Transmisión:** Poleas dentadas GT2 para sistemas de transmisión (donde se requiera alejar el peso de los eslabones distales), rodamientos radiales para absorber las cargas axiales/radiales mitigando la fricción teórica ($\tau_{\text{friction}}$), y anillos de retención (Seeger circlips) para fijar los ejes estructurales de manera segura.

## 3.6 CAD Modeling & Assembly Constraints

El paso de la formulación analítica al software de diseño asistido por computadora (CAD) está condicionado por las siguientes reglas restrictivas derivadas de la simulación:

1.  **Parámetros de Fabricación Aditiva:** Para que la resistencia estructural final corresponda a la matriz analítica de deflexión ($\delta$), las piezas impresas (PLA/ABS) deben parametrizarse en laminadores como PrusaSlicer o Cura respetando el 30% de infill cúbico. Además, se debe aumentar el grosor de las paredes perimetrales (shell thickness) específicamente en las uniones de los motores J1 y J2 para mitigar la fatiga por concentración de esfuerzos.
2.  **Alineación de Ejes:** Como dictamina la matriz de rotación, la articulación de la base debe ser estrictamente vertical. Los planos CAD deben garantizar tolerancias precisas (H7/g6) para los rodamientos de apoyo de la base, previniendo cabeceos durante las aceleraciones altas.
3.  **Distribución de Masas (Inercia):** El diseño debe integrar la placa del ESP32-S3 y la etapa de potencia lo más cerca posible del punto de anclaje de J0 o en un compartimento estacionario externo. Esto previene un aumento parasitario en el parámetro $m_i$ de los eslabones móviles que colapsaría el cálculo del momento de inercia ($I_{\text{total}}$).
