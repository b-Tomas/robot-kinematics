#import "template.typ": *

#show: project.with(
  title: "Cinemática de robots",
  subtitle: "Transformaciones lineales aplicadas a robots articulados",
  authors: (
    (name: "Badenes, Tomás", mail: "tomasbadenes@alu.ing.unlp.edu.ar"),
    (name: "Fernández, Santiago Adriel", mail: "fernandez.adriel@alu.ing.unlp.edu.ar"),
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

Para solucionar esto, se utilizan las llamadas _coordenadas homogéneas_ @cox_little_oshea[cap. 8], que hacen uso de vectores de una dimensión más y los proyectan sobre el subespacio deseado. Al igual que las transformaciones lineales tradicionales nos permiten escalar e inclinar los ejes, y además nos permite trasladarlos (entre otras opciones).

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
  text("Traslación")(x) = mat(
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

La ventaja de este método es que los parámetros son fáciles de obtener. Al momento de modelar, se piensa gráficamente qué rotaciones y traslaciones nos permiten transformar el origen de un sistema de referencia en otro. Particularmente, se suele pensar en la secuencia
1. rotación en torno al eje $z$ ($theta_i$),
2. traslación sobre el eje $z$ ($d_i$),
3. traslación sobre el eje $x$ ($r_i$)
4. y rotación en torno al eje $x$ ($alpha_i$)
que sufre el origen de ${i-1}$ para transformarse en el origen de ${i}$ #footnote[Una visualización del método de obtención de estos parámetros se puede encontrar en #underline(link("https://youtu.be/rA9tm0gTln8", "youtu.be/rA9tm0gTln8")).]. Nótese que la matriz que se obtiene es exactamente la inversa de esta transformación.

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

El robot consta de cuatro rotores. Así, los parámetros de Denavit–Hartenberg se pueden pensar gráficamente de la siguiente manera:

1. El primer rotor rota en torno al eje $z$ de la base, por lo que $theta_1 = q_1$ variable. Luego, se traslada $L_1$ sobre el eje $z$ para ubicarse en el origen del siguiente rotor, por lo que $d_1 = L_1$ constante. Como no hay traslaciones sobre el eje $x$, $r_1 = 0$. Finalmente, como el eje de rotación del siguiente rotor es paralelo la piso, se realiza una rotación de $-frac(pi, 2)$ sobre el eje $x$ para que el eje $z$ (eje sobre el que se rotará el siguiente rotor) quede paralelo al piso, por lo que $alpha_1 = -frac(pi,2)$ constante. Nótese que también se podría rotar sobre $frac(pi, 2)$, pero se eligió esta configuración porque el modelo del brazo, como se muestra en la figura, rota con ángulos positivos de manera horaria.
2. El segundo rotor también rotará en torno a su eje $z$, por lo que $theta_2 = q_2$ variable. Sin embargo, tal y como están planteados los ejes, la posición inicial del segmento 2 es paralelo al plano, por lo que hay que "levantarlo" para que quede como en la figura. Así, se agrega un desface $phi > 0$ tal que $theta_2 = q_2 - phi$. A simple vista, se aprecia que $phi approx frac(pi,2)$, aunque en realidad el valor es ligeramente menor. Como el origen del próximo rotor está sobre el mismo plano que éste, el eje $z$ se mantiene igual, por lo que $d_2 = alpha_2 = 0$. Finalmente, se traslada $L_2$ sobre el eje $x$ para ubicarse en el origen del siguiente rotor, por lo que $r_2 = L_2$ constante.
3. Similarmente, el tercer rotor también rotará en torno a su eje $z$, por lo que $theta_3 = q_3$ variable. También se tiene en cuenta el desface anteriormente mencionado, solo que esta vez se aplica su opuesto, quedando $theta_3 = q_3 + phi$. Como el origen del próximo rotor está sobre el mismo plano que este, el eje $z$ se mantiene tal cual, por lo que $d_3 = alpha_3 = 0$. Finalmente, se traslada $L_3$ sobre el eje $x$ para ubicarse en el origen del siguiente rotor, por lo que $r_3 = L_3$ constante.
4. Por último, el cuarto rotor también rotará en torno a su eje $z$, por lo que $theta_4 = q_4$ variable. Ahora, para llegar al _end-effector_ (que no es más que un punto arbritario, no un rotor), solo hace falta trasladarse $L_4$ sobre el eje $x$, por lo que $r_4 = L_4$ constante y $d_4 = alpha_4 = 0$. 

#figure(
  table(
    columns: (auto, auto, auto, auto, auto),
    inset: 10pt,
    align: center,
    [*Enlace*], $theta_i$,            $d_i$,  $r_i$,  $alpha_i$,
    [1],        $q_1$,                $L_1$,  $0$,    $-frac(pi,2)$,
    [2],        $q_2 - phi$,  $0$,    $L_2$,  $0$,
    [3],        $q_3 + phi$,  $0$,    $L_3$,  $0$,
    [4],        $q_4$,                $0$,    $L_4$,  $0$,
  ),
  caption: [Parámetros de Denavit–Hartenberg obtenidos.]
)

Así, cada una de las matrices intermedias quedan

$
  T^1_0 (q_1) = 
  T_(r z)(q_1) T_(z)(L_1) T_(r x)(-frac(pi,2)) =
  mat(
    cos(q_1),  0, -sin(q_1),   0;
    sin(q_1),  0,  cos(q_1),   0;
           0, -1,         0, L_1;
           0,  0,         0,   1;
  )
$

$
  T^2_1 (q_2) =
  T_(r z) (q_2-phi)T_(x) (L_2) =
  mat(
    cos(q_2-phi), -sin(q_2-phi), 0, L_2cos(q_2-phi);
    sin(q_2-phi),  cos(q_2-phi), 0, L_2sin(q_2-phi);
               0,             0, 1,               0;
               0,             0, 0,               1;
  )
$

$
  T^3_2 (q_3) =
  T_(r z) (q_3+phi)T_(x) (L_3) =
  mat(
    cos(q_3+phi), -sin(q_3+phi), 0, L_3cos(q_3+phi);
    sin(q_3+phi),  cos(q_3+phi), 0, L_3sin(q_3+phi);
               0,             0, 1,               0;
               0,             0, 0,               1;
  )
$

$
  T^4_3 (q_4) =
  T_(r z) (q_4)T_(x) (L_4) =
  mat(
    cos(q_4), -sin(q_4), 0, L_4cos(q_4);
    sin(q_4),  cos(q_4), 0, L_4sin(q_4);
           0,         0, 1,           0;
           0,         0, 0,           1;
  )
$

Luego de todo esto, finalmente se puede obtener la posición del _end-effector_ con la función:

$
  P(q_1, q_2, q_3, q_4) &= (T^1_0 (q_1) space T^2_1 (q_2) space T^3_2 (q_3) space T^4_3 (q_4)) space (0,0,0,1)^T \
  &= T^4_0 (q_1,q_2,q_3,q_4) space (0,0,0,1)^T 
$

= Inverse kinematics -- Obteniendo los parámetros

Es muy interesante aprender sobre _forward kinematics_, pero realmente no tiene demasiadas aplicaciones. En el espacio de la robótica resulta esencial poder realizar el procedimiento inverso: averiguar los ángulos necesarios para que el brazo se ubique en una posición deseada. Obtener esta función inversa se llama _inverse kinematics_ @wangrobotics[cap. 3].

Al buscar la función inversa, no siempre se podrá obtener en forma analítica, ya que hay configuraciones de robots que permiten varios (o hasta infinitos) conjuntos de ángulos para alcanzar un punto. En ese caso, se suele el método de aproximación de Newton–Raphson para obtener una solución numérica.

En el caso del _OpenManipulator X_, se logró obtener una solución analítica.

== Obteniendo la solución

Para empezar, se obtiene la forma analítica de $T^4_0$, que se puede dividir en dos partes más sencillas: $T^4_0 = T^1_0 space T^4_1$. $T^4_1$, haciendo un poco de trigonometría, se obtiene:

$
  omega = q_2 + q_3 + q_4
  \
  T^4_1(q_2,q_3,q_4) = mat(
    cos(omega), -sin(omega), 0, L_2cos(q_2-phi) + L_3cos(q_2+q_3) + L_4cos(omega);
    sin(omega),  cos(omega), 0, L_2sin(q_2-phi) + L_3sin(q_2+q_3) + L_4sin(omega);
             0,           0, 1,                                                 0;
             0,           0, 0,                                                 1;
  )
$

Multiplicando por $T^1_0$ se obtiene

$
  T^4_0(q_1,q_2,q_3,q_4) = \
  mat(
    cos(q_1)cos(omega), -cos(q_1)sin(omega), -sin(q_1), L_2cos(q_1)cos(q_2-phi) + L_3cos(q_1)cos(q_2+q_3) + L_4cos(q_1)cos(omega);
    sin(q_1)cos(omega), -sin(q_1)sin(omega),  cos(q_1), L_2sin(q_1)cos(q_2-phi) + L_3sin(q_1)cos(q_2+q_3) + L_4sin(q_1)cos(omega);
           -sin(omega),         -cos(omega),         0,                   L_1 - L_2sin(q_2-phi) - L_3sin(q_2+q_3) - L_4sin(omega);
                     0,                   0,         0,                                                                         1;
  )
$

Esta monstruosidad que se sale de los márgenes del papel es la matriz de transformación que nos permite permite obtener la posición del _end-effector_ a partir de los parámetros del robot (la función $P(q_1,q_2,q_3,q_4)$ definida más arriba). Particularmente, nos queda la siguiente solución:

$
  vec(x,y,z,1) = T^4_0(q_1,q_2,q_3,q_4) vec(0,0,0,1) = vec(
    cos(q_1)(L_2cos(q_2-phi) + L_3cos(q_2+q_3) + L_4cos(omega)),
    sin(q_1)(L_2cos(q_2-phi) + L_3cos(q_2+q_3) + L_4cos(omega)),
    L_1 - L_2sin(q_2-phi) - L_3sin(q_2+q_3) - L_4sin(omega),
    1
  )
$

Además de las coordenadas $(x,y,z)$ que se quieren alcanzar, para resolver el sistema de ecuaciones también tendremos el ángulo de ataque del brazo. Esto es, cuando el robot se mueva a un punto, puede llegar a este de varios ángulos distintos. No casualmente, este ángulo será $omega$. Gráficamente, se puede deducir que $omega = q_2 + q_3 + q_4$, es decir, la suma de todas las rotaciones. Además, como $q_1$ solo rota sobre el eje $z$ no afecta a este ángulo. (Nótese que $omega$ positivo es cuando el _end-effector_ rota en sentido horario, y negativo cuando rota en sentido antihorario.)

Así, nos queda el sistema de ecuaciones:

$
  vec(x,y,z,omega) = vec(
    cos(q_1)(L_2cos(q_2-phi) + L_3cos(q_2+q_3) + L_4cos(omega)),
    sin(q_1)(L_2cos(q_2-phi) + L_3cos(q_2+q_3) + L_4cos(omega)),
    L_1 - L_2sin(q_2-phi) - L_3sin(q_2+q_3) - L_4sin(omega),
    q_2 + q_3 + q_4,
  )
$

Dividiendo las primeras dos ecuaciones, se obtiene

$
  frac(y,x) &= frac(sin(q_1),cos(q_1)) = tan(q_1) \
        q_1 &= arctan(frac(y,x))
$

Obtenido $q_1$ #footnote[Al momento de obtener el $q_1$ computacionalmente, se utiliza la función #text(font: "Cascadia Mono", "atan2(y,x)") que tiene en consideración el caso $x=0$ y retorna el ángulo correspondiente a su lugar en el plano en vez de un ángulo entre $-pi/2$ y $pi/2$.], veremos que lo podemos eliminar del sistema de la siguiente manera:

$
  sqrt(x^2 + y^2) &= sqrt((cos(q_1)^2 + sin(q_1)^2)(L_2cos(q_2-phi) + L_3cos(q_2+q_3) + L_4cos(omega))^2) \
                  &= sqrt((L_2cos(q_2-phi) + L_3cos(q_2+q_3) + L_4cos(omega))^2) \
 ±sqrt(x^2 + y^2) &= L_2cos(q_2-phi) + L_3cos(q_2+q_3) + L_4cos(omega)
$

Así, se reduce el sistema a

$
  cases(
    ±sqrt(x^2 + y^2) &= L_2cos(q_2-phi) + L_3cos(q_2+q_3) + L_4cos(omega),
    z &= L_1 - L_2sin(q_2-phi) - L_3sin(q_2+q_3) - L_4sin(omega),
    omega &= q_2 + q_3 + q_4,
  )
$

¡que no es más que un problema de $RR^2$! Lo cual tiene sentido, ya que los rotores 2, 3 y 4 están contenidos en un mismo plano. Resolviendo el sistema con métodos trigonométricos #footnote[Una explicación paso a paso puede verse en #underline(link("https://youtu.be/1-FJhmey7vk", "youtu.be/1-FJhmey7vk")).], los ángulos finales quedan:

$
  r   &= ±sqrt(x^2 + y^2) - L_4cos(omega) \
  h   &= L_1 - z - L_4sin(omega) \
  \
  q_2 &= phi + arctan(frac(h, r)) ± arccos(frac(r^2 + h^2 + L_2^2 - L_3^2, 2L_2 sqrt(r^2 + h^2))) \
  q_3 &= arctan(frac(h-L_2sin(q_2 - phi), r-L_2cos(q_2 - phi))) - q_2 \
  q_4 &= omega - q_2 - q_3
$

Esto no queda de forma matricial porque las soluciones no siempre son únicas y cuando no lo son suelen ser finitas, principalmente porque hay funciones trigonométricas de por medio.

Así, para aplicar los contenidos tratados, planteamos controlar una simulación del robot OpenMANIPULATOR-X a partir del desarrollo obtenido anteriormente.

= Simulando el robot

El proyecto práctico realizado aplicando la teoría anterior se basa en ROS #footnote(underline(link("https://www.ros.org"))) (_Robot Operating System_), un set de herramientas ampliamente utilizado en el campo de la robótica para el desarrollo de software. Se creó un entorno basado en Docker para facilitar la portabilidad del proyecto en el que se incluye la simulación en Gazebo #footnote(underline(link("https://gazebosim.org/home"))) provista por Robotis y el programa que se desarrolló.

Este programa es un CLI (_command-line interface_) permite el ingreso de coordenadas cartesianas que son transformadas a parámetros y enviadas al robot. Esta CLI puede ser utilizada tanto para controlar el robot simulado como un robot real. La función de ingreso de posición recibe cuatro datos, la posición en los ejes $x$, $y$ y $z$ y el ángulo del _end-effector_ al momento de llegar a la posición destino.

El proyecto desarrollado incluyendo el entorno de simulación se encuentra disponible en #link("https://github.com/b-Tomas/robot-kinematics", "github.com/b-Tomas/robot-kinematics") de manera abierta.

#v(2cm)

#bibliography(style: "apa", "bibliography.bib")

#place(
  bottom,
)[ 
  #line(length: 100%)

  Este documento fue creado con Typst #sys.version. \
  Última modificación realizada el #datetime.today().display("[day]/[month]/[year]").
]