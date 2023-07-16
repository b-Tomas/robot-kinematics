#import "template.typ": *

#show: project.with(
  title: "Cinemática de robots",
  subtitle: "Transformaciones lineales aplicadas a robots articulados",
  authors: (
    (name: "Badenes, Tomás", mail: "tomasbadenes@alu.ing.unlp.edu.ar"),
    (name: "Fernández, Santiago Adriel", mail: "TBD"),
    (name: "Majoros, Lorenzo", mail: "lorenzomajoros@alu.ing.unlp.edu.ar"),
    (name: "Seery, Juan Martín", mail: "juan.seery@alu.ing.unlp.edu.ar"),
  )
)

// Simple macro to add editing notes
#let note(txt) = text(fill: red, weight: "bold", txt);

= Introducción

La cinemática de robots es la ciencia que estudia el movimiento de cadenas articuladas con múltiples grados de libertad. Se asocia con el estudio de cuerpos rígidos y hace fuerte uso de la geometría. Las técnicas descritas por esta disciplina se aplican en todo tipo de ámbitos, desde animaciones por computadora hasta la biomecánica. Excelentes ejemplos son los brazos utilizados en líneas de ensamblaje o los brazos robot utilizados para operar en el exterior de la estacion espacial internacional.

Aquí se tratará la solo parte más superficial del área: obtener ecuaciones que relacionan un punto en el espacio con, en este caso, rotaciones de motores (los llamados parámetros del robot). Esto es esencial, ya que nos permitiría saber cuánto hay que rotar los motores para que el robot llegue a un punto, a dónde está apuntando en un momento dado, cuál es el alcance del mismo, entre un sinfín de otros ejemplos.

#pad(
  top: 30pt,
  figure(
    image("images/robot_photo.png"),
    caption: [Brazo OpenMANIPULATOR-X]
  )
)

Se verá cómo modelar y controlar a partir de estas operaciones el robot *OpenMANIPULATOR-X* #footnote(underline(link("https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview"))), un brazo robótico de 4 grados de libertad creado por la empresa ROBOTIS #footnote(underline(link("https://en.robotis.com"))) para el proyecto TurtleBot 3 #footnote(underline(link("https://www.turtlebot.com/turtlebot3/"))) en conjunto a Open Robotics #footnote(underline(link("https://www.openrobotics.org"))). Se eligió este robot porque es de plataforma, hardware y software abierto, y provee un entorno de simulación. Además, está pensado como proyecto educativo y viene acompañado de extensa documentación, lo cual resulta ideal para el proyecto.


= Forward kinematics -- ¿a dónde estoy apuntando?

Un brazo robótico no es más que un conjunto de articulaciones unidas con enlaces. Para empezar, nos gustaría poder mover el robot. Físicamente, se necesita pasarle electrónicamente a cada uno de los rotores cuántos grados se quiere que roten y los mismos rotarán. ¿Cómo se podría calcular el punto en el que terminará el extremo del brazo dados los ángulos de los motores?

Para ejemplificar, se proponen tres puntos articulados ${0}, {1}, {2}$ unidos por enlaces $L_1, L_2$. $L_i$, numéricamente, es la distancia entre ${i-1}$ e ${i}$. ${0}$ se encuentra fija al suelo, y ${0}$ y ${1}$ pueden rotar sobre un plano. En este ejemplo (y en el robot utilizado) solamente se tienen articulaciones que rotan, lo cual simplifica bastante la teoría necesaria. Estas articulaciones también podrían expandirse, moverse de manera esférica o incluso helicoidal, que por suerte no es el caso.

#figure(
  image("images/image1.png", width: 70%),
  caption: [
    Puntos articulados en el plano XY.
  ],
)

Ahora, a cada articulación ${i}$ se le asignará su propio sistema de coordenadas dado por la base arbitraria $B_i = {bold(upright(hat(x)))_i, bold(upright(hat(y)))_i}$. Cuando ${i}$ rote, también lo harán sus ejes y, por consiguiente, se moverán y trasladarán todos los ejes de las articulaciones ${j}: j>i$.

#figure(
  image("images/image2.png", width: 70%),
  caption: [
    Los puntos articulados en el plano con sus respectivos ejes asociados.
  ],
)

#let T = $T^i_(i-1)$;

Así, la estrategia será plantear transformaciones que nos permitan pasar de uno de estos sistemas (o espacios) a otro. Así, cuando se rote uno de los espacios, podremos ver dónde terminan los espacios siguientes.

Se plantean las transformaciones #T, que nos permiten pasar de la "base" $B_i$ a la "base" $B_(i-1)$. Pero esto es imposible. Estas transformaciones necesitan que se mueva el $bold(0)$ de un espacio al siguiente, lo cual no es una transformación lineal.

== Coordenadas homogéneas

Para solucionar esto, se utilizan las llamadas _coordenadas homogéneas_ @cox_little_oshea[cap. 8], que hacen uso de vectores de una dimensión más y los proyectan sobre el subespacio deseado. Al igual que las transformaciones lineales tradicionales nos permiten escalar e inclinar los ejes, y además nos permite transladarlos (entre otras opciones).

Este tópico es muy extenso y admite muchos tipos de proyecciones. En nustro caso, se utilizarán matrices de este tipo:

$
  H = mat( bold(R), bold(d); bold(0), 1 ) in RR^((n+1)times(n+1))
$

$bold(R) in RR^(n times n)$ es análoga a una matriz transformación sobre el subespacio proyectado: representa escalación, rotación, inclinación, etc. $bold(d) in RR^n$ es un vector que representa un desplazamiento del origen del subespacio. Ejemplos prácticos en $RR^2$ son:

$
  text("Rotación")(theta) = mat(
    cos(theta), -sin(theta), 0;
    sin(theta),  cos(theta), 0;
             0,           0, 1;
  )
  \ \
  text("Translación")(x) = mat(
    1, 0, x;
    0, 1, 0;
    0, 0, 1;
  )
$

que nos permiten rotar los ejes y trasladarlos en dirección el eje $x$ respectivamente. Finalmente, para obtener los vectores de los subespacios transformados, se aplica

$
  bold(upright(v')) &= H bold(upright(v)) \
  vec(x_1',x_2', dots.v, x_n', 1) &= H vec(x_1,x_2, dots.v, x_n, 1)
$

quedando el vector deseado con una componente extra que será siempre $1$. Cabe mencionar nuevamente que esta componente simpre queda $1$ en este caso particular, y no para toda transformación en coordenadas homogéneas.

== Parámetros de Denavit–Hartenberg

Ahora que tenemos un método para movernos del sistema de referencia de una articulación a la anterior con #T, siendo esta traslaciones y rotaciones en el coordenadas homogéneas, la matriz transformación que nos "lleva" o nos "cambia de base" de la última unión a la base será:

$
  T^n_0 &= T^1_0 space T^2_1 space dots.h space T^n_(n-1)
$

¿Por qué se querría esta matriz? Porque $T^n_0 dot (0,...,0,1)^T$ será igual al origen de la punta de nuestro brazo (llamado _end-effector_) pero en las coordenadas de la base del robot (es decir, de nuestro espacio canónico). Nos permite saber dónde está posicionado el robot.

Se llama _forward kinematics_ a este campo de estudio @wangrobotics[cap. 2]. Hay varios métodos para obtener las matrices #T, siendo el más popular en $RR^3$ el de Denavit–Hartenberg, conocido como _parámetros de Denavit–Hartenberg_. Consta de representar cada una de las matrices #T en base a cuatro parámetros: una rotación en torno al eje $z$ ($theta$), una traslación sobre el eje $z$ ($d$), una traslación sobre el eje $x$ ($r$) y una rotación en torno al eje $x$ ($alpha$):

$
  #T &= T_(r z)(theta_i)T_(z)(d_i)T_(x)(r_i)T_(r x)(alpha_i) \
     &= mat(
          cos(theta_i), -sin(theta_i), 0, 0;
          sin(theta_i),  cos(theta_i), 0, 0;
                     0,             0, 1, 0;
                     0,             0, 0, 1;
        )
        mat(
          1, 0, 0, 0;
          0, 1, 0, 0;
          0, 0, 1, d_i;
          0, 0, 0, 1;
        )
        mat(
          1, 0, 0, alpha_i;
          0, 1, 0, 0;
          0, 0, 1, 0;
          0, 0, 0, 1;
        )
        mat(
          1,            0,             0, 0;
          0, cos(alpha_i), -sin(alpha_i), 0;
          0, sin(alpha_i),  cos(alpha_i), 0;
          0,            0,             0, 1;
        ) \
     &= mat(
       cos(theta_i), -sin(theta_i)cos(alpha_i),  sin(theta_i)sin(alpha_i), r_i cos(theta_i);
       sin(theta_i),  cos(theta_i)cos(alpha_i), -cos(theta_i)sin(alpha_i), r_i sin(theta_i);
                  0,              sin(alpha_i),              cos(alpha_i),              d_i;
                  0,                         0,                         0,                1;
     )
$

La ventaja de este método es que los parámetros son fáciles de obtener #footnote[Una visualización del método de obtención de estos parámetros se puede encontrar en #underline(link("https://youtu.be/rA9tm0gTln8", "youtu.be/rA9tm0gTln8")).] y el método sirve para la gran mayoría de casos.

Ahora, estas matrices #T son estáticas. Se necesita que varíen según ángulos arbritarios. Las articulaciones del robot _OpenManipulator X_, como se planteó anteriormente, solamente rotan. Se puede plantear que todos nuestros motores rotan en torno a sus respectivos ejes $z$ y que sus matrices sean

$
  #T (q_i) = T_(r z)(q_i + theta_i)T_(z)(d_i)T_(x)(r_i)T_(r x)(alpha_i)
$

siendo $q_i$ los parámetros del robot y $theta_i$ las condiciones iniciales.


== Modelando el _OpenMANIPULATOR-X_

#figure(
  image("images/robot_joints.png", width: 70%),
  caption: [
    Representación digital del robot con anotaciones.
  ],
)

El robot consta de cuatro rotores. Estos se han modelado de tal forma que sus parámetros de Denavit–Hartenberg son los siguientes:

#figure(
  table(
    columns: (auto, auto, auto, auto, auto),
    inset: 10pt,
    align: center,
    [*Enlace*], $d_i$, $theta_i$, $r_i$, $alpha_i$,
    [1], $L_1$, $q_1$, $0$, $frac(pi,2)$,
    [2], $0$, $q_2 + frac(pi, 2)$, $L_2$, $0$,
    [3], $0$, $q_3 - frac(pi, 2)$, $L_3$, $0$,
    [4], $0$, $q_4$, $L_4$, $0$,
  ),
  caption: [Parámetros de Denavit–Hartenberg.]
)
siendo $L_i$ la longitud del segmento _i_.

Así, cada una de las matrices intermedias quedan

$
T^1_0 (q_1) = T_(r z)(q_1)T_(z)(L_1)T_(r x)(frac(pi,2)) =mat(
  cos(q_1), 0, sin(q_1),  0;
  sin(q_1), 0, -cos(q_1), 0;
  0,        1, 0,         L_1;
  0,        0, 0,         1;
)
$

$
T^2_1 (q_2) = T_(r z)(q_2 + frac(pi,2))T_(x)(L_2) =mat(
  -sin(q_2), -cos(q_2), 0, -L_2sin(q_2);
  cos(q_2),  -sin(q_2), 0, L_2cos(q_2);
  0,         0,         1, 0;
  0,         0,         0, 1;
)
$

$
T^3_2 (q_3) = T_(r z)(q_3 - frac(pi,2))T_(x)(L_3) =mat(
  sin(q_3), cos(q_3), 0, L_3sin(q_3);
  -cos(q_3),  sin(q_3), 0, -L_3cos(q_3);
  0,         0,         1, 0;
  0,         0,         0, 1;
)
$

$
T^4_3 (q_4) = T_(r z)(q_4)T_(x)(L_4) =mat(
  cos(q_4), -sin(q_4), 0, L_4cos(q_4);
  sin(q_4), cos(q_4),  0, L_4sin(q_4);
  0,        0,         1, 0;
  0,        0,         0, 1;
)
$

Luego de todo esto, finalmente se puede obtener la posición del _end-effector_ con la función:

$
  P(q_1, q_2, q_3, q_4) = (T^1_0 (q_1) space T^2_1 (q_2) space T^3_2 (q_3) space T^4_3 (q_4)) space (0,0,0,1)^T
$

= Inverse kinematics -- Obteniendo los parámetros

Es muy interesante aprender sobre _forward kinematics_, pero realmente no tiene demasiadas aplicaciones. En el espacio de la robótica resulta esencial poder realizar el procedimiento inverso: averiguar los ángulos necesarios para que el brazo se ubique en una posición deseada. Obtener esta función inversa se llama _inverse kinematics_ @wangrobotics[cap. 3].

Al buscar la función inversa, no siempre se podrá obtener en forma analítica, ya que hay configuraciones de robots que permiten varios (o hasta infinitos) conjuntos de ángulos para alcanzar un punto. En ese caso, se suele el método de aproximación de Newton–Raphson para obtener una solución numérica.

En el caso del _OpenManipulator X_, se logró obtener una solución analítica.

== Obteniendo la solución

Este problema se puede dividir en dos problemas más sencillos. El ángulo de la primera articulación $q_1$, la que está en la base, solo depende las coordenadas $x$ e $y$. Particularmente,

$
  q_1 = arctan(frac(y,x))
$

Para el resto de ángulos $q_2$, $q_3$ y $q_3$, se puede aprovechar el hecho de que los tres se mueven sobre un mismo plano perpendicular al plano $x y$. Así, podemos hacer un cambio de variables:

$
  x' &= sqrt(x^2 + y^2) \
  y' &= z
$

Así podemos obtener la matriz de tranformación $T^n_1$ a partir de los parámetros de Denavit–Hartenberg presentados en la tabla 1, (obviando $T^1_0$) pero modificada para que funcione con vectores de $RR^2$ en la base dada por los ejes $x'$ e $y'$.

$
  omega = q_2 + q_3 + q_4
  \
  T^n_1'(q_2,q_3,q_4) = mat(
    cos(omega), -sin(omega), L_2cos(q_2) + L_3cos(q_2+q_3) + L_4cos(omega);
    sin(omega),  cos(omega), L_2sin(q_2) + L_3sin(q_2+q_3) + L_4sin(omega);
                   0,                 0,                                                   1;
  )
  \
  bold(upright(v)) = T^n_1'(q_2,q_3,q_4) dot (x',y',1)^T
$

$omega$ representa el ángulo de ataque del robot. Así, nos queda un sistema de tres incógnitas:

$
  omega &= q_2 + q_3 + q_4 \
  x'    &= L_2cos(q_2) + L_3cos(q_2+q_3) + L_4cos(omega) \
  y'    &= L_2sin(q_2) + L_3sin(q_2+q_3) + L_4sin(omega) 
$

Resolviendo #footnote[Una explicación paso a paso puede verse en #underline(link("https://youtu.be/1-FJhmey7vk", "youtu.be/1-FJhmey7vk")).], los ángulos finales quedan

$
  x'' &= x'-L_4cos(omega) \
  y'' &= y'-L_4sin(omega) \
  q_2 &= arctan(frac(y'', x'')) ± arccos(frac((x'')^2 + (y'')^2 + (L_2)^2 - (L_3)^2, sqrt((x'')^2 + (y'')^2))) \
  q_3 &= arctan(frac(y''-L_2sin(q_2), x''-L_2cos(q_2))) - q_2 \
  q_4 &= omega - q_2 - q_3
$

Esto no queda de forma matricial porque las soluciones no siempre son únicas y cuando no lo son suelen ser finitas, principalmente porque hay funciones trigonométricas de por medio.

Así, para aplicar los contenidos tratados, planteamos controlar una simulación del robot OpenMANIPULATOR-X a partir del desarrollo obtenido anteriormente.

= Simulando el robot 

El proyecto práctico realizado aplicando la teoría anterior se basa en ROS #footnote(underline(link("https://www.ros.org"))) (_Robot Operating System_), un set de herramientas ampliamente utilizado en el campo de la robótica para el desarrollo de software. Se creó un entorno basado en Docker para facilitar la portabilidad del proyecto en el que se incluye la simulación en Gazebo #footnote(underline(link("https://gazebosim.org/home"))) provista por Robotis y el programa que se desarrolló.

Este programa es un CLI (_command-line interface_) permite el ingreso de coordenadas cartesianas que son transformadas a parámetros y enviadas al robot. Esta CLI puede ser utilizada tanto para controlar el robot simulado como un robot real. La función de ingreso de posición recibe cuatro datos, la posición en los ejes $x$, $y$ y $z$ y el ángulo del _end-effector_ al momento de llegar a la posición destino.

El proyecto desarrollado incluyendo el entorno de simulación se encuentra disponible en #link("https://github.com/b-Tomas/robot-kinematics", "github.com/b-Tomas/robot-kinematics") de manera abierta.

#bibliography(style: "apa", "bibliography.yml")
