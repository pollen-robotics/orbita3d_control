---
title: Kinematics
layout: default
---


WARNING:
> This page is a work in progress and is not yet complete.

# Orbita kinematics

The Orbita 3D actuator is a parallel mechanism composed of 3 active arms and 3 passive arms. The active arms are driven by motors and the passive arms are connected to the platform. The platform is the output of the mechanism and can be inclined in 3D space.

This page describes a mathematical model of the Orbita mechanism, which allows the calculation of the platform's position, velocity, and torque based on the motor positions.

NOTE: 
>Some references to understand the kinematics of Orbita: <br>
>   - "On the direct kinematics of a class of spherical three-degree-of freedom parallel manipulators", Gosselin et al. 1992<br>
>   - "Kinematic analysis, optimization and programming of parallel robotic manipulator", Gosselin 1985

## Model definition

<img src="../../img/orbita_kinematics.png" alt="axis" style="width: 400px;"/>


Schéma du \(i\)-ème bras d'Orbita partant du moteur \(M_i\) (rotation autour du vecteur unitaire \(\vec{u_i}\)), le premier bras "montant" (rotation autour du vecteur unitaire \(\vec{w_i}\)) et le bras "coudé" (rotation autour du vecteur unitaire \(\vec{v_i}\)).

\(\theta_i\) correspond à l'angle de rotation du moteur (par rapport au repère fixe \(O X_0 Y_0 Z_0\)).

\(\phi_i\) correspont à l'angle de rotation du bras "coudé" par rapport au bras "montant"

On note que la rotation autour de \(\vec{v_i}\) n'a pas besoin d'être connue, les 3 vecteurs \(\vec{v_i}\) permettent de déterminer l'orientation de la plateforme.

## Cinematique inverse

On utilise ici la convention [Denavit-Hartenberg](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters) pour définir les repères correspondant à chaque articulation.

Le repère \(O X_{i1} Y_{i1} Z_{i1}\) du moteur est confondu avec le repère fixe \(O X_0 Y_0 Z_0\) pour \(i=1,2,3\)

Si on suit la convention, les paramètres \(a\) et \(b\) de Denavit-Hartenberg sont nuls.
Les \(\alpha\) (angles entre les axes \(Z_{i,n}\) consécutifs) valent:

$$\widehat{Z_{i1}Z_{i2}}=\pi-\alpha_1$$

$$\widehat{Z_{i2}Z_{i3}}=\alpha_2$$

\(\alpha_1\) et \(\alpha_2\) sont des paramètre mécaniques, pour Orbita v1 on a: \(\alpha_1=50°\) et \(\alpha_2=90°\)

Les \(\theta\) (angles de rotation autour des axes \(Z_{i,n}\)) valents:

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


On a des \(\cos\theta_i\) et \(\sin\theta_i\), ce qui n'est pas pratique. Mais du coup on peut utiliser le trick de la tangente du demi angle avec:

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


### Example python implementation

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

## Jacobienne

NOTE:
Ref pour certains détails de calcul: "On the Numerical Solution of the Inverse Kinematic Problem", Angeles 1980.

La Jacobienne \(J\) est la matrice des dérivées partielles qui lie les vitesses articulaires aux vitesses angulaires (cartésiennes) d'Orbita. En temps normal pour un mécanisme série on étudie la Jacobienne \(J\) qui donne: \(J\dot{\theta}=\omega\), mais comme dans le cas des mécanismes parallèles le problème de la cinématique inverse est beaucoup plus simple que la directe, on étudie ici la Jacobienne inverse, que l'on va quand même appeler \(J\).

On a: \(J\omega=\dot{\theta}\) avec \(\omega\) le (pseudo-)vecteur vitesse angulaire (qui est l'axe de rotation instantané et dont la norme représente la vitesse) et \(\dot{\theta}\) la vitesse "articulaire" (les moteurs).

On a vu précédemment que la cinématique inverse était obtenue avec: \(w_i\cdot v_i = \cos\alpha_2\)

Si on dérive cette expression de chaque coté on a: \(\dot{w_i}\cdot v_i+w_i \cdot \dot{v_i} = 0\)

$$-v_{i1}\sin \alpha_1\sin\theta_i + v_{i2}\sin\alpha_1\cos\theta_i$$

On a vu aussi que: \(v_i=Qu_i\), avec \(Q\) la matrice de rotation entre une configuration "initiale" \(u_i\) et la configuration de la plateforme \(v_i\) (rappel: les \(v_i\) sont les 3 vecteurs de la plateforme)

Si on défini \(\dot{Q}=\Omega Q\) avec \(\Omega\) la matrice anti-symétrique telle que: \(\Omega \equiv \frac{\partial (\omega \times a)}{\partial a}\) pour tout \(a\). 

On a \(Vect(\Omega)=\omega\)

D'après la notation de Angeles (1980), \(Vect(A)\) étant le vecteur (3D) invariant de A.

NOTE:
> Du coup \(Vect(Q)\) est vaguement l'axe de rotation, le lien avec le vecteur rotation unitaire \(u\) au sens de Euler (représentation axe-angle): \(Vect(Q)=u.\sin\phi\) avec \(\phi\) l'angle de rotation autour de \(u\).
>
> Angeles (1980) défini \(Vect(A)=\left[\frac{1}{2}\epsilon_{ijk}.a_{kj}\right]_{i,j,k \in 1,2,3}^T\)
> avec 
> $$A = \begin{bmatrix} a_{11} & a_{12} & a_{13} \\ a_{21} & a_{22} & a_{23} \\ a_{31} & a_{32} & a_{33}\end{bmatrix} $$ et \(\epsilon_{ijk}\) le symbole de "Levi-Civita", en suivant la notation d'Einstein, soit en 3D:
> $$\begin{equation}
    \epsilon_{ijk}=
    \begin{cases}
        +1\;si\: (i,j,k)\: est\: (1,2,3)\: ou\: (2,3,1)\: ou\: (3,1,2)\\
        -1\;si\: (i,j,k)\: est\: (3,2,1)\: ou\: (1,3,2)\: ou\: (2,1,3)\\
         0\;si\: i=j\: ou\: j=k\: ou\: k=i  
    \end{cases}
    \end{equation}$$
> Donc 
> $$Vect(A)= \frac{1}{2}\begin{bmatrix} \epsilon_{123}a_{32}+\epsilon_{132}a_{23}\\ \epsilon_{213}a_{31}+\epsilon_{231}a_{13}\\\epsilon_{321}a_{12}+\epsilon_{312}a_{21} \end{bmatrix} = \frac{1}{2}\begin{bmatrix} a_{32}-a_{23}\\ a_{13}-a_{31}\\a_{21}-a_{12} \end{bmatrix}$$
>
>\(\Omega\) est le "tenseur de vitesse angulaire", c'est à dire une matrice de "rotation infinitésimale".
>
>$$\Omega=\begin{bmatrix} 0 & -\omega_z & \omega_y\\ \omega_z & 0 & -\omega_x\\ -\omega_y & \omega_x& 0 \end{bmatrix}$$, on a bien \(Vect(\Omega)=\omega\)
>
>Avec du coup \(\omega \times r= \Omega \cdot r\), \(r\) un vecteur quelconque subissant une rotation.

Si on écrit la dérivée des vecteurs \(w_i\):

$$ w_i  = \begin{bmatrix} \sin{\alpha_1}\cos{\theta_i} \\ \sin{alpha_1}\sin{\theta_i} \\ -\cos{\alpha_1}\end{bmatrix}, \qquad v_i = \begin{bmatrix}  v_{i1}\\ v_{i2} \\ v_{i3}\end{bmatrix} , \qquad \frac{\partial w_i}{\partial \theta_i} = \begin{bmatrix} -\sin{\alpha_1}\sin{\theta_i}\\  \sin{\alpha_1}\cos{\theta_i} \\ 0 \end{bmatrix}$$

On a donc: $$\frac{1}{\dot{\theta_i}} \dot{w_i} = \begin{bmatrix} -\sin\alpha_1 \sin\theta_i \\ \sin\alpha_1 \cos\theta_i \\ 0 \end{bmatrix}=u_i\times w_i$$ avec dans le cas Orbita (moteurs colinéaires) \(u_i=s=[0, ~0, ~1 ]^T\)

Donc si on dérive \(v_i=Qu_i\) et avec \(\dot{Q}=\Omega Q\) on a:

\(\dot{v_i}=\dot{Q}u_i=\Omega Q u_i=\Omega v_i\)

Donc en injectant dans \(\dot{w_i}\cdot v_i+w_i\cdot \dot{v_i}=0\) on a 

\(\dot{\theta_i}(u_i \times w_i)\cdot v_i+w_i \cdot \Omega v_i=0\)

\(\dot{\theta_i}(u_i \times w_i)\cdot v_i - \omega \cdot (w_i\times v_i)=0\)

Donc:

\(\dot{\theta_i}=\frac{(w_i \times v_i)\cdot \omega}{(u_i \times w_i)\cdot v_i}\)

Comme on a \(J\omega=\dot{\theta}\), la \(i\)-ème ligne \(J_i\) de la Jacobienne est:

\(J_i=\frac{w_i\times v_i}{(u_i \times w_i)\cdot v_i}=\frac{w_i\times v_i}{(s \times w_i)\cdot v_i}\)

La Jacobienne complete est du coup:


$$
J = \left[\begin{matrix}\frac{\sin{\left(\alpha_{1} \right)} \sin{\left({\theta}_{1} \right)} {v}_{1,3} + \cos{\left(\alpha_{1} \right)} {v}_{1,2}}{- \sin{\left(\alpha_{1} \right)} \sin{\left({\theta}_{1} \right)} {v}_{1,1} + \sin{\left(\alpha_{1} \right)} \cos{\left({\theta}_{1} \right)} {v}_{1,2}} & \frac{- \sin{\left(\alpha_{1} \right)} \cos{\left({\theta}_{1} \right)} {v}_{1,3} - \cos{\left(\alpha_{1} \right)} {v}_{1,1}}{- \sin{\left(\alpha_{1} \right)} \sin{\left({\theta}_{1} \right)} {v}_{1,1} + \sin{\left(\alpha_{1} \right)} \cos{\left({\theta}_{1} \right)} {v}_{1,2}} & 1\\\frac{\sin{\left(\alpha_{1} \right)} \sin{\left({\theta}_{2} \right)} {v}_{2,3} + \cos{\left(\alpha_{1} \right)} {v}_{2,2}}{- \sin{\left(\alpha_{1} \right)} \sin{\left({\theta}_{2} \right)} {v}_{2,1} + \sin{\left(\alpha_{1} \right)} \cos{\left({\theta}_{2} \right)} {v}_{2,2}} & \frac{- \sin{\left(\alpha_{1} \right)} \cos{\left({\theta}_{2} \right)} {v}_{2,3} - \cos{\left(\alpha_{1} \right)} {v}_{2,1}}{- \sin{\left(\alpha_{1} \right)} \sin{\left({\theta}_{2} \right)} {v}_{2,1} + \sin{\left(\alpha_{1} \right)} \cos{\left({\theta}_{2} \right)} {v}_{2,2}} & 1\\\frac{\sin{\left(\alpha_{1} \right)} \sin{\left({\theta}_{3} \right)} {v}_{3,3} + \cos{\left(\alpha_{1} \right)} {v}_{3,2}}{- \sin{\left(\alpha_{1} \right)} \sin{\left({\theta}_{3} \right)} {v}_{3,1} + \sin{\left(\alpha_{1} \right)} \cos{\left({\theta}_{3} \right)} {v}_{3,2}} & \frac{- \sin{\left(\alpha_{1} \right)} \cos{\left({\theta}_{3} \right)} {v}_{3,3} - \cos{\left(\alpha_{1} \right)} {v}_{3,1}}{- \sin{\left(\alpha_{1} \right)} \sin{\left({\theta}_{3} \right)} {v}_{3,1} + \sin{\left(\alpha_{1} \right)} \cos{\left({\theta}_{3} \right)} {v}_{3,2}} & 1\end{matrix}\right]
$$

On note que la Jacobienne dépend de \(\alpha_1\) mais pas de \(\alpha_2\). Elle dépend de l'orientation de la plateforme mais aussi des \(\theta_i\).

### Example python implementation

```python
def get_jacobian(v,thetas,alpha1=np.radians(50)):
    sa1=np.sin(alpha1)
    ca1=np.cos(alpha1)
    st1=np.sin(thetas[0])
    ct1=np.cos(thetas[0])
    st2=np.sin(thetas[1])
    ct2=np.cos(thetas[1])
    st3=np.sin(thetas[2])
    ct3=np.cos(thetas[2])
    
    row1_denom=-sa1*st1*v[0][0]+sa1*ct1*v[0][1]
    row2_denom=-sa1*st2*v[1][0]+sa1*ct2*v[1][1]
    row3_denom=-sa1*st3*v[2][0]+sa1*ct3*v[2][1]
    
    row1=[(sa1*st1*v[0][2]+ca1*v[0][1])/row1_denom, (-sa1*ct1*v[0][2]-ca1*v[0][0])/row1_denom,1]
    row2=[(sa1*st2*v[1][2]+ca1*v[1][1])/row2_denom, (-sa1*ct2*v[1][2]-ca1*v[1][0])/row2_denom,1]
    row3=[(sa1*st3*v[2][2]+ca1*v[2][1])/row3_denom, (-sa1*ct3*v[2][2]-ca1*v[2][0])/row3_denom,1]
    return np.array([row1,row2,row3])
```

## Cinématique Directe

On cherche à déterminer l'orientation de la plateforme (les \(v_i\)) en fonction des \(\theta_i\). 

Pour ça on peut se contenter de trouver les \(\phi_i\) vu qu'on a vu précédemment (cf. cinématique inverse) que: \(v_i=RQ_{i1}Q_{i2}e\) avec \(R=I\) matrice identité et \(e=[0,0,1]^T\)

Par souci de simplicité on va définir \(\sin \theta_1=st1\), \(\cos \phi_1=cp1\), \(\cos \alpha_1 = ca1\)...

$$
v_1 =\displaystyle \left[\begin{matrix}- ca_{1} cp_{1} sa_{2} st_{1} + ca_{2} sa_{1} st_{1} + ct_{1} sa_{2} sp_{1}\\ca_{1} cp_{1} ct_{1} sa_{2} - ca_{2} ct_{1} sa_{1} + sa_{2} sp_{1} st_{1}\\- ca_{1} ca_{2} - cp_{1} sa_{1} sa_{2}\end{matrix}\right]
$$

$$ 
v_2 = \displaystyle \left[\begin{matrix}- ca_{1} cp_{1} sa_{2} st_{1} + ca_{2} sa_{1} st_{1} + ct_{1} sa_{2} sp_{1}\\ca_{1} cp_{1} ct_{1} sa_{2} - ca_{2} ct_{1} sa_{1} + sa_{2} sp_{1} st_{1}\\- ca_{1} ca_{2} - cp_{1} sa_{1} sa_{2}\end{matrix}\right]
$$

$$ 
v_3 =  \displaystyle \left[\begin{matrix}- ca_{1} cp_{1} sa_{2} st_{1} + ca_{2} sa_{1} st_{1} + ct_{1} sa_{2} sp_{1}\\ca_{1} cp_{1} ct_{1} sa_{2} - ca_{2} ct_{1} sa_{1} + sa_{2} sp_{1} st_{1}\\- ca_{1} ca_{2} - cp_{1} sa_{1} sa_{2}\end{matrix}\right]
$$

On a comme conditions:

\(w_i \cdot v_i=\cos\alpha_2\) qu'on a vu précédemment.

\(v_i \cdot v_j=-\frac{1}{2}\) pour \(i\neq j\) et \(i,j=1,2,3\) puisque les \(v_i\) sont à 120° deux à deux.

\(||v_i||=1\) puisque les \(v_i\) sont des vecteurs unité

et

\(v_1+v_2+v_3=0\) puisque les \(v_i\) sont sur le même plan et à 120°.

Donc on va faire un gros système d'équations avec tout ça et résoudre numériquement

### Example python implementation

```python
def forward_kinematics(thetas, initial_guess=(0,1,0,1,0,1), yaw_offset=0):
    disk0=np.array([0,np.radians(120),np.radians(-120)])
    #thetas+=disk0
    theta1,theta2,theta3=thetas
    p_subs={sa1:sin(rad(50)), ca1:cos(rad(50)), sa2:sin(rad(90)), ca2:cos(rad(90)),ct1:cos(theta1),st1:sin(theta1),ct2:cos(theta2),st2:sin(theta2),ct3:cos(theta3),st3:sin(theta3)}
    eqs=[]
    eqs=[Eq(v(1).dot(v(2)).expand().subs(p_subs),-1.0/2.0),Eq(v(2).dot(v(3)).expand().subs(p_subs),-1.0/2.0),Eq(v(3).dot(v(1)).expand().subs(p_subs),-1.0/2.0)]
    eqs.append(Eq(w(1).dot(v(1)),ca2).subs(p_subs))
    eqs.append(Eq(w(2).dot(v(2)),ca2).subs(p_subs))
    eqs.append(Eq(w(3).dot(v(3)),ca2).subs(p_subs))
    eqs.append(Eq(v(1).dot(v(1)),1).subs(p_subs))
    eqs.append(Eq(v(2).dot(v(2)),1).subs(p_subs))
    eqs.append(Eq(v(3).dot(v(3)),1).subs(p_subs))
    eqs.append((v(1)+v(2)+v(3)).subs(p_subs))
    eqs=[e for e in eqs if e not in [True, False] ]
    
    sols=nsolve(eqs,(cp1,sp1,cp2,sp2,cp3,sp3),initial_guess,dict=True,tol=1e-10)
    
    sp1_n=float(sols[0][sp1])
    sp2_n=float(sols[0][sp2])
    sp3_n=float(sols[0][sp3])

    cp1_n=float(sols[0][cp1])
    cp2_n=float(sols[0][cp2])
    cp3_n=float(sols[0][cp3])

    phi1=np.arctan2(sp1_n,cp1_n)
    phi2=np.arctan2(sp2_n,cp2_n)
    phi3=np.arctan2(sp3_n,cp3_n)
    print(f'phi1: {phi1} ({np.degrees(phi1)}°) phi2: {phi2} ({np.degrees(phi2)}°) phi3: {phi3} ({np.degrees(phi3)}°)')
    res_subs={sa1:sin(rad(50)), ca1:cos(rad(50)), sa2:sin(rad(90)), ca2:cos(rad(90)),cp1:cos(phi1),sp1:sin(phi1),ct1:cos(theta1),st1:sin(theta1),cp2:cos(phi2),sp2:sin(phi2),ct2:cos(theta2),st2:sin(theta2),cp3:cos(phi3),sp3:sin(phi3),ct3:cos(theta3),st3:sin(theta3)}

    v1sol=np.array(Matrix(v(1).subs(res_subs)).evalf())
    v2sol=np.array(Matrix(v(2).subs(res_subs)).evalf())
    v3sol=np.array(Matrix(v(3).subs(res_subs)).evalf())
    #donc bizarement on a [y, -x, z] (rotation de 90°)
    v1sol=np.array([-v1sol[1][0],v1sol[0][0],v1sol[2][0]],dtype=np.float64)
    v2sol=np.array([-v2sol[1][0],v2sol[0][0],v2sol[2][0]],dtype=np.float64)
    v3sol=np.array([-v3sol[1][0],v3sol[0][0],v3sol[2][0]],dtype=np.float64)
    
    V_mat=np.matrix([v1sol,v2sol,v3sol])
    #config de base
    b1=np.array([np.cos(np.radians(0)),np.sin(np.radians(0)),0])
    b2=np.array([np.cos(np.radians(120)),np.sin(np.radians(120)),0])
    b3=np.array([np.cos(np.radians(-120)),np.sin(np.radians(-120)),0])
    
    #dans le repère Orbita:
    yaw_offset=-np.pi/2.0
    #yaw_offset=0
    roffset=Rotation.from_euler('xyz',[0,0,-yaw_offset])
    Qoffset=np.array(roffset.as_matrix())
    b1=Qoffset.dot(b1)
    b2=Qoffset.dot(b2)
    b3=Qoffset.dot(b3)
    B_mat=np.matrix([b1,b2,b3])
    print(B_mat)
    print(V_mat)
    ra,rms=Rotation.align_vectors(V_mat,B_mat)
    return ra.as_euler('xyz', degrees=True)
```