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

## 3.1 Torque Analysis & Motor Sizing

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

Para elegir las dimensiones optimas del modelo, se crearon simulaciones en codigo simulando el torque en cualquier pose elegida con la formula anterior, ignorando la friccion viscosa tau debido a que no es posible medirla, se entiende que es significativa. Elcodigo usado, en los anexos se resume en las siguientes formulas. 

Cinemática Directa (Transformaciones Homogéneas): El algoritmo ubica cada articulación, centro de masa y punto final (TCP) en un espacio 3D multiplicando matrices de rotación y traslación. Esto permite calcular las distancias reales sin importar qué tan compleja sea la postura del brazo.Torque mediante Producto Cruz: Para adaptar el modelo al espacio 3D, el cálculo proyectado por el coseno se sustituye por el producto cruz entre el vector de distancia (desde el motor hasta el centro de masa o carga) y el vector de gravedad. Solo se toma la magnitud de la fuerza que actúa directamente sobre el eje de rotación de cada motor.Cálculo de Inercia y Aceleración Límite: El programa calcula el momento de inercia proyectando las masas de forma perpendicular al eje de giro. Al restarle al torque nominal del motor el esfuerzo requerido para sostener el brazo (estático), se obtiene el torque dinámico disponible. Con este remanente, el código despeja la aceleración angular máxima ($\alpha$) que el brazo puede soportar antes de fallar, identificando automáticamente la articulación que funciona como "cuello de botella".


## 3.2 Kinematic & Dynamic Constraints
(Sección vacía - Pendiente por definir)

## 3.3 Structural & Material Resistance
(Sección vacía - Pendiente por definir)

## 3.4 Bill of Materials (BOM) & Component Selection
(Sección vacía - Pendiente por definir)

## 3.5 CAD Modeling & Assembly Constraints
(Sección vacía - Pendiente por definir)


