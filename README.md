# CC-project
In this repository there's a University project that me and a colleague of mine have done for an exam focused on Linear Control for MIMO systems.

## Description
The model that we have considered for this project is *2-DOF Helicopter* which is a fourth-order nonlinear dynamical system with 2 inputs and 2 outputs. In this project the aim is to give a suitable and desired motion around *pitch* axis and *yaw* axis taking into account that these 2 dynamics are coupled. That's why is so difficult controlling an helicopter!


### Outline
The project is structured in the following way:
1. We started showing how to derive dynamical model according to the Lagrangian approach, we have linearized such model around an equilibrium point and we have checked for this linear MIMO system the *reachable* and *observable* properties.
2. First controllers which we have designed are state feedback controllers with pole placement technique. Gradually we have added a gain matrix in feedforward, then an integral action in order to obtain a zero error at steady-state and finally a Luemberger observer under the assumption that state is unknown.
3. The second kind of controller which we have designed is always a state feedback controller, but now the feedback gain has been designed with LQ optimal control. Also in this case, in order to satisfy control requirments, especially at steady-state, we added a feedforward gain matrix, an integral action, and, since now there's an LQ optimal control, the observer that we had to design is been the Kalman filter.
4. After we have designed controllers exploiting the state-space representation of the model, at the end we choose to design a $H_{\infty}$ control, so exploiting a representation of the model in the frequency-domain.

For more details you can read my technical report named as **technical report.pdf**. It has been written in Italian though. Sorry for this.

## Instructions for use
If you want to see the source code used for the project, then visit the folder called **Script MATLAB & Simulink**. If you want to run those files, then make sure that you've installed following MATLAB toolbox:
- Simulink
- Control System toolbox