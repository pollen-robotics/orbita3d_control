---
title: Kinematics
layout: default
---


## Orbita kinematics

The Orbita 3D actuator is a parallel mechanism composed of 3 active arms and 3 passive arms. The active arms are driven by motors and the passive arms are connected to the platform. The platform is the output of the mechanism and can be inclined in 3D space.

This page describes a mathematical model of the Orbita mechanism, which allows the calculation of the platform's position, velocity, and torque based on the motor positions.

NOTE: 
>Some references to understand the kinematics of Orbita: <br>
>   - "On the direct kinematics of a class of spherical three-degree-of freedom parallel manipulators", Gosselin et al. 1992<br>
>   - "Kinematic analysis, optimization and programming of parallel robotic manipulator", Gosselin 1985

### Model definition

<img src="../../img/orbita_kinematics.png" alt="axis" style="width: 400px;"/>


Schéma du \(i\)-ème bras d'Orbita partant du moteur \(M_i\) (rotation autour du vecteur unitaire \(\vec{u_i}\)), le premier bras "montant" (rotation autour du vecteur unitaire \(\vec{w_i}\)) et le bras "coudé" (rotation autour du vecteur unitaire \(\vec{v_i}\)).

\(\theta_i\) correspond à l'angle de rotation du moteur (par rapport au repère fixe \(O X_0 Y_0 Z_0\)).

\(\phi_i\) correspont à l'angle de rotation du bras "coudé" par rapport au bras "montant"

On note que la rotation autour de \(\vec{v_i}\) n'a pas besoin d'être connue, les 3 vecteurs \(\vec{v_i}\) permettent de déterminer l'orientation de la plateforme.


On utilise ici la convention [Denavit-Hartenberg](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters) pour définir les repères correspondant à chaque articulation.

Le repère \(O X_{i1} Y_{i1} Z_{i1}\) du moteur est confondu avec le repère fixe \(O X_0 Y_0 Z_0\) pour \(i=1,2,3\)

Si on suit la convention, les paramètres \(a\) et \(b\) de Denavit-Hartenberg sont nuls.
Les \(\alpha\) (angles entre les axes \(Z_{i,n}\) consécutifs) valent:

$$\widehat{Z_{i1}Z_{i2}}=\pi-\alpha_1$$

$$\widehat{Z_{i2}Z_{i3}}=\alpha_2$$

\(\alpha_1\) et \(\alpha_2\) sont des paramètre mécaniques, pour Orbita v1 on a: \(\alpha_1=50°\) et \(\alpha_2=90°\)

Les $\theta$ (angles de rotation autour des axes $Z_{i,n}$) valents:

$$\hat{Z_{i1}}=\theta_i$$

$$\hat{Z_{i2}}=\phi_i$$

Si on ne considère que l'orientation, on obtient une matrice de rotation \(Q_{i1}\) pour passer de \(O X_{i1} Y_{i1} Z_{i1}\) vers \(O X_{i2} Y_{i2} Z_{i2}\):

$$Q_{i1}=\begin{bmatrix}
\cos\theta_i & \cos\alpha_1 \sin\theta_i & \sin\alpha_1\sin\theta_i \\
\sin\theta_i & -\cos\alpha_1\cos\theta_i & -\sin\alpha_1 \cos\theta_i \\
0 & \sin\alpha_1 & -\cos\alpha_1 \\
\end{bmatrix}$$

De même la matrice de rotation passant de \(O X_{i2} Y_{i2} Z_{i2}\) vers \(O X_{i3} Y_{i3} Z_{i3}\):

$$Q_{i2}=\begin{bmatrix}
\cos\phi_i & -\cos\alpha_2 \sin\phi_i & \sin\alpha_2\sin\phi_i \\
\sin\phi_i & \cos\alpha_2\cos\phi_i & -\sin\alpha_2 \cos\phi_i \\
0 & \sin\alpha_2 & \cos\alpha_2 \\
\end{bmatrix}$$

Du coup le vecteur $$\vec{w_i}=Q_{i1}\cdot \vec{u_i}=Q_{i1}\cdot\begin{bmatrix} 0\\0\\1\end{bmatrix}=\begin{bmatrix}\sin\alpha_1 \sin\theta_i\\ -\sin\alpha_1 \cos\theta_i \\ -\cos\alpha_1\end{bmatrix}$$

Le problème de la cinématique inverse revient à trouver les \(\theta_i\) connaissant l'orientation de la plateforme. Sachant que l'orientation de la plateforme est déterminée entièrement par les vecteurs \(\vec{v_i}\).

Or on a  \(\vec{w_i}\cdot\vec{v_i}=||\vec{w_i}||\cdot||\vec{v_i}||\cdot \cos\alpha_2=\cos\alpha_2\) pour \(i=1,2,3\)

Il faut donc résoudre le système:

$$
    \begin{cases}
      \vec{w_1}\cdot\vec{v_1}=\cos\alpha_2\\
      \vec{w_2}\cdot\vec{v_2}=\cos\alpha_2\\
      \vec{w_3}\cdot\vec{v_3}=\cos\alpha_2
    \end{cases}
$$

Donc si $$\vec{v_i}=\begin{bmatrix} v_{i1}\\ v_{i2} \\ v_{i3} \end{bmatrix}$$

On note qu'on a aussi: 
$$\vec{v_i}=Q_{i1}Q_{i2}\vec{u_i}=\begin{bmatrix} -\cos\alpha_1\cdot \cos\phi_i \cdot \sin\alpha_2 \cdot \sin\theta_i + \cos\alpha_2 \cdot \sin\alpha_1 \cdot \sin\theta_i + \cos\theta_i \cdot \sin\alpha_2 \cdot \sin\phi_i \\ \cos\alpha_1 \cdot \cos\phi_i \cdot \sin\alpha_2 \cdot \cos\theta_i - \cos\alpha_2 \cdot \sin\alpha_1 \cdot \cos\theta_i + \sin\theta_i \cdot \sin\alpha_2 \cdot \sin\phi_i \\ -\cos\alpha_1 \cdot \cos\alpha_2 - \cos\phi_i \cdot \sin\alpha_1 \cdot \sin\alpha_2 \end{bmatrix}$$


On a des $\cos\theta_i$ et $\sin\theta_i$, ce qui n'est pas pratique. Mais du coup on peut utiliser le trick de la tangente du demi angle avec:

$$T_i=\tan\frac{\theta_i}{2}$$ et donc

$$\sin\theta_i=\frac{2T_i}{1+T_i^2}$$

$$\cos\theta_i=\frac{1-T_i^2}{1+T_i^2}$$

on tombe bien sur la même expression que Gosselin:

\begin{equation}
A_iT_i^2+2B_iT_i+C_i=0
\end{equation}
avec

$$A_i=-v_{i1}\sin\alpha_1-v_{i3}\cos\alpha_1-\cos\alpha_2$$

$$B_i=v_{i2}\sin\alpha_1$$

$$C_i=v_{i1}\sin\alpha_1-v_{i3}\cos\alpha_1-\cos\alpha_2$$

donc avec des solutions de a forme:

$$T_i=\frac{-B_i \pm \sqrt{B_i^2-A_iC_i}}{A_i}$$, pour $$i=1,2,3$$

On a donc une combinaison de 2 solutions par "bras" ce qui fait 8 configurations possibles. 


### Example python IK implementation
```python
def SolIK(v, alpha1, alpha2):
    T_i=[]
    for i in range(3):
        Ai=-np.sin(alpha1)*v[i][0]-np.cos(alpha1)*v[i][2]-np.cos(alpha2)
        Bi=np.sin(alpha1)*v[i][1]
        Ci=np.sin(alpha1)*v[i][0]-np.cos(alpha1)*v[i][2]-np.cos(alpha2)
        sol1=(-Bi+np.sqrt(Bi**2-Ai*Ci))/Ai
        sol2=(-Bi-np.sqrt(Bi**2-Ai*Ci))/Ai
        T_i.append([sol1,sol2])
    return T_i
```