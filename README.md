# 3DOF-Manipulator-Simulation

## 🎯 Objetivo
Desenvolver um mini-projeto formal em robótica:
Simulação de um manipulador planar 3-DOF com modelação cinemática e controlo proporcional.

## 📚 Estrutura Técnica
### 1️⃣ Introdução
- Objetivo do projeto
- Motivação (interesse em robótica)
- Scope: apenas cinemática (sem dinâmica)

### 2️⃣ Modelação Geométrica
- Definição dos comprimentos L1,L2,L3
- Forward Kinematics (equações trigonométricas)
- Justificação por usar abordagem geométrica (simplicidade planar)

### 3️⃣ Inverse Kinematics
- Derivação analítica
- Múltiplas soluções (elbow-up / elbow-down)
- Limitações

### 4️⃣ Jacobiana
- Derivação a partir da FK
- Relação 𝑥˙=𝐽(𝑞)𝑞˙
- Interpretação física

### 5️⃣ Singularidades
- Condição matemática
- Interpretação física (braço estendido, perda de direção)
- Visualização

### 6️⃣ Workspace
- Região alcançável
- Interpretação geométrica

### 7️⃣ Trajectory Tracking
- Definir trajetória (linha ou círculo)
- Controlador proporcional (cinemático)
- Resultados da simulação
- Pequeno vídeo

### 8️⃣ Discussão
- Limitações
- Ausência de dinâmica
- Possível extensão futura (PID, torque-level control)
