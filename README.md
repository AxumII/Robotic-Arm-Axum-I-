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

Before CAD modeling and physical assembly, analytical models were evaluated to determine the physical constraints, structural requirements, and dynamic limits of the robotic arm.

## 3.1 Calculos generales para el torque 

To ensure the actuators can support both static holding loads and dynamic accelerations, torque requirements were evaluated at each joint under maximum cantilever conditions (fully extended horizontal pose):

$$
\tau_{\text{joint}} = \sum_{i} \left( m_i \cdot g \cdot r_i \cdot \cos(\theta_i) \right) + I_{\text{total}} \cdot \alpha + \tau_{\text{friction}}
$$

Where:
* $\tau_{\text{joint}}$: Total required torque at the joint axis.
* $m_i$: Mass of link $i$ and mounted hardware/payload.
* $g$: Gravitational acceleration ($9.81 \text{ m/s}^2$).
* $r_i$: Distance from the joint axis to the center of mass of link $i$.
* $\theta_i$: Absolute angular orientation relative to the horizontal plane ($\cos(0^\circ) = 1$ for worst-case cantilever position).
* $I_{\text{total}}$: Total moment of inertia referred to the joint rotation axis.
* $\alpha$: Required angular acceleration ($\ddot{\theta}$).
* $\tau_{\text{friction}}$: Estimated viscous and Coulomb friction losses in bearings and gearboxes.

Para elegir las dimensiones optimas del modelo, se crearon simulaciones en codigo simulando el torque en cualquier pose elegida con la formula anterior, ignorando la friccion viscosa tau debido a que no es posible medirla, se entiende que es significativa. 
El script llamado : es el usado para este menester del calculo del torque necesario para diferentes dimensiones del brazo con diversos motores. 

Inicialmente se plantean las distancias en un modelo 3D.

1. Matrices de Transformación Homogénea (Helpers)Para calcular las posiciones de las articulaciones independientemente de la postura del brazo, el código utiliza Transformaciones Homogéneas (4x4). Estas matrices combinan rotación y traslación en una sola operación matricial.El código define funciones (rot_x, rot_y, rot_z) que generan las siguientes matrices de rotación dependiendo del eje sobre el que gire el motor:$$R_x(\theta) = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & \cos\theta & -\sin\theta & 0 \\ 0 & \sin\theta & \cos\theta & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$$$R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta & 0 \\ 0 & 1 & 0 & 0 \\ -\sin\theta & 0 & \cos\theta & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$$$R_z(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta & 0 & 0 \\ \sin\theta & \cos\theta & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$Y la matriz de traslación (translation) para los desplazamientos (offsets y centros de masa):$$\text{Trans}(v_x, v_y, v_z) = \begin{bmatrix} 1 & 0 & 0 & v_x \\ 0 & 1 & 0 & v_y \\ 0 & 0 & 1 & v_z \\ 0 & 0 & 0 & 1 \end{bmatrix}$$2. Cinemática Directa (forward_kinematics)Esta función calcula la posición exacta en el espacio de cada articulación, su eje de giro (vector Z local) y los centros de masa. Lo logra multiplicando secuencialmente las matrices desde la base hasta la herramienta (TCP):$$T_i = T_{i-1} \cdot R_{\text{eje}}(\theta_i) \cdot \text{Trans}(\text{offset}_i)$$De la matriz resultante $T_i$, el código extrae:Posición de la articulación ($P_{\text{axis}}$): La columna de traslación (primeras 3 filas, 4ta columna).Vector del eje de giro ($\vec{u}_{\text{axis}}$): El vector direccional correspondiente a X, Y o Z de la matriz de rotación.3. Cálculo de Torques Estáticos (calculate_torques_and_limits)En un entorno 3D, el torque estático no se calcula con un simple coseno, sino mediante el producto cruz ($\times$) entre el vector de distancia radial ($\vec{r}$) y el vector de fuerza gravitacional ($\vec{F}_g$).Para cada articulación $i$, se evalúa el peso de todos los eslabones $j$ y cargas posteriores:Vector de distancia: $\vec{r} = P_{\text{masa}} - P_{\text{axis}}$Torque tridimensional total:$$\vec{\tau}_{3D} = \sum_{j>i} \left( \vec{r}_j \times (m_j \cdot \vec{g}) \right) + (\vec{r}_{\text{load}} \times (m_{\text{load}} \cdot \vec{g}))$$Proyección del Torque: Los motores solo sufren esfuerzo en su eje de rotación. El código aísla esta fuerza usando el producto punto ($\cdot$) entre el torque 3D y el vector unitario del eje de giro del motor ($\vec{u}_{\text{axis}}$):$$\tau_{\text{static}} = \vert{}\vec{\tau}_{3D} \cdot \vec{u}_{\text{axis}}\vert{}$$4. Cálculo de Inercia y Restricciones DinámicasPara saber si el motor puede acelerar la carga sin quemarse, el código calcula el momento de inercia ($I$). La inercia depende de la distancia perpendicular de las masas al eje de rotación. El código calcula esta distancia proyectando vectores:Componente paralela al eje:$$\vec{r}_{\parallel} = (\vec{r} \cdot \vec{u}_{\text{axis}}) \vec{u}_{\text{axis}}$$Distancia perpendicular real:$$r_{\perp} = \vert{}\vert{}\vec{r} - \vec{r}_{\parallel}\vert{}\vert{}$$Inercia total en la articulación:$$I_{\text{total}} = \sum \left( m_j \cdot r_{\perp, j}^2 \right)$$Finalmente, el código verifica si el motor tiene capacidad sobrante (torque dinámico) después de soportar el peso estático. Con este remanente, despeja la aceleración máxima que esa articulación puede soportar utilizando la Segunda Ley de Newton para la rotación:$$\alpha_{\text{max}} = \frac{\tau_{\text{rated}} - \tau_{\text{static}}}{I_{\text{total}}}$$Donde $\tau_{\text{rated}}$ es el torque nominal del servo (convertido de kg-cm a N-m mediante las constantes declaradas en el constructor de la clase). Si la aceleración objetivo (alpha_target) excede el valor mínimo calculado de $\alpha_{\text{max}}$ en cualquiera de las articulaciones, el sistema arroja una advertencia (cuello de botella).

## 3.2 Torque Analysis & Motor Sizing

Frente a las restricciones de torque, estas dependen de los motores disponibles con un torque dado para velocidades angulares bajas, de las cargas de los segmentos y del propio peso de los motores. Además, es necesario considerar el torque del motor tanto en dinámico como en estático, de modo que sea eficiente de acuerdo a su durabilidad interna. Teniendo en cuenta esto, se ejecutaron varias simulaciones y se obtuvieron los siguientes rangos (ver tablas generadas por el script de simulación).

## 3.3 Kinematic & Dynamic Constraints

Dado el torque anterior, también tenemos restricciones dinámicas, siendo la principal restricción las velocidades angulares. Estas, como se mencionó, deben ser bajas tanto para aprovechar un mayor torque (debido a la mayor relación de engranajes), como por la no necesidad de velocidades altas; ya que los brazos, aparte de requerir precisión, al tener un alcance inferior a 50 cm no exigen velocidades lineales altas. Además, a mayor velocidad se requeriría mayor aceleración, lo que dificultaría frenar el brazo al cambiar de poses, convirtiendo una inercia alta en un problema relevante. 

Otra restricción crítica es la aceleración angular máxima ($\alpha_{\text{max}}$) dictada por el torque dinámico disponible y el momento de inercia total del sistema:

$$ \alpha_{\text{max}} = \frac{\tau_{\text{nominal}} - \tau_{\text{estático}}}{I_{\text{total}}} $$

Esta aceleración debe ser lo suficientemente alta para mover el brazo con soltura, pero lo suficientemente baja para evitar problemas de frenado y mantener los requerimientos de inercia acotados. También se evaluaron casos de diseño empleando sistemas de poleas para mitigar el peso en los eslabones distales. 
Dadas las restricciones del inciso anterior, el código anexo genera las tablas que muestran los parámetros de aceleraciones angulares resultantes para cada postura.

## 3.4 Structural & Material Resistance

Para esta sección se analizan los posibles materiales para el brazo, teniendo en cuenta las cargas estáticas y dinámicas anteriores. En general, el cálculo geométrico se asume con secciones transversales de 30x50 mm. Dadas las restricciones monetarias y la disponibilidad de métodos de fabricación (Impresión 3D y corte láser), se evaluaron tres materiales principales: MDF, PLA y ABS. 

Se evaluaron las distancias cinemáticas con perfiles de estos materiales. Para el caso específico del MDF, se adaptó el cálculo para usar láminas con secciones de 8x40 mm. Los segmentos de brazos más cortos a calcular son de 20 mm (con muñecas de 10 mm), mientras que los máximos evaluados alcanzan los 45 cm (con muñecas de 25 mm)[cite: 1]. 

Los resultados comparativos de masa e inercia según el material elegido alimentan la simulación y se resumen en la siguiente tabla de resultados arrojada por el script.

## 3.5 Bill of Materials (BOM) & Component Selection
Dado los resultados de los calculos anteriores 

## 3.6 CAD Modeling & Assembly Constraints
(Sección vacía - Pendiente por definir)


